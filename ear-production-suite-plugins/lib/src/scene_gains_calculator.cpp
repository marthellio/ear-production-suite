#include "scene_gains_calculator.hpp"
#include "ear/metadata.hpp"
#include "helper/eps_to_ear_metadata_converter.hpp"
#include <future>
#include <algorithm>

namespace ear {
namespace plugin {

SceneGainsCalculator::SceneGainsCalculator(ear::Layout outputLayout,
                                           int inputChannelCount)
    : objectCalculator_{outputLayout},
      directSpeakersCalculator_{outputLayout},
      hoaCalculator_{outputLayout} {
  resize(outputLayout, static_cast<std::size_t>(inputChannelCount));

  std::lock_guard<std::mutex> lock(commonDefinitionHelperMutex_);
  commonDefinitionHelper.getElementRelationships();
}

void SceneGainsCalculator::update(proto::SceneStore store) {
  // Called by NNG callback on thread with small stack.
  // Launch task in another thread to overcome stack limitation.

  {
    std::lock_guard<std::mutex> lock(storeToProcessMutex_);
    bool jobAlreadyScheduled = storeToProcess.has_value(); // If it already had a value, then a job would have already been set up
    storeToProcess = store;
    if(jobAlreadyScheduled) return;
  }

  auto future = std::async(std::launch::async, [this]() {

    std::lock_guard<std::mutex> lockG(gainVectorsMutex_);
    std::lock_guard<std::mutex> lockS(storeToProcessMutex_);

    {
      std::lock_guard<std::mutex> lockR(routingCacheMutex_);
      for(auto id : removedIds(*storeToProcess)) {
        auto routing = routingCache_[id];
        for(int i = 0; i < routing.size; ++i) {
          std::fill(direct_[routing.track + i].begin(),
                    direct_[routing.track + i].end(), 0.0f);
          std::fill(diffuse_[routing.track + i].begin(),
                    diffuse_[routing.track + i].end(), 0.0f);
        }
        routingCache_.erase(id);
      }
      for(auto routing : updateRoutingCache(*storeToProcess)) {
        for(int i = 0; i < routing.size; ++i) {
          std::fill(direct_[routing.track + i].begin(),
                    direct_[routing.track + i].end(), 0.0f);
          std::fill(diffuse_[routing.track + i].begin(),
                    diffuse_[routing.track + i].end(), 0.0f);
        }
      }
    }

    std::lock_guard<std::mutex> lockC(gainCalculatorsMutex_);

    for (const auto& item : storeToProcess->monitoring_items()) {

      bool newItem = true;
      {
        std::lock_guard<std::mutex> lockA(allActiveIdsMutex_);
        newItem = std::find(allActiveIds.begin(), allActiveIds.end(), item.connection_id()) == allActiveIds.end();
      }

      if (newItem || item.changed()) {
        if (item.has_ds_metadata()) {
          auto earMetadata =
              EpsToEarMetadataConverter::convert(item.ds_metadata());
          auto routing = static_cast<std::size_t>(item.routing());
          if (item.routing() >= 0 &&
              routing + earMetadata.size() < direct_.size()) {
            for (int i = 0; i < earMetadata.size(); ++i) {
              directSpeakersCalculator_.calculate(earMetadata.at(i),
                                                  direct_[routing + i]);
            }
          }
        }
        if (item.has_mtx_metadata()) {
          throw std::runtime_error("received unsupported Matrix type metadata");
        }
        if (item.has_obj_metadata()) {
          auto earMetadata =
              EpsToEarMetadataConverter::convert(item.obj_metadata());
          auto routing = static_cast<std::size_t>(item.routing());
          if (item.routing() >= 0 && routing < direct_.size()) {
            objectCalculator_.calculate(earMetadata, direct_[routing],
                                        diffuse_[routing]);
          }
        }
        if (item.has_hoa_metadata()) {
          ear::HOATypeMetadata earMetadata;
          {
            std::lock_guard<std::mutex> lock(commonDefinitionHelperMutex_);
            earMetadata = EpsToEarMetadataConverter::convert(
                item.hoa_metadata(), commonDefinitionHelper);
          }

          std::vector<std::vector<float>> hoaGains(
              earMetadata.degrees.size(),
              std::vector<float>(direct_[0].size(), 0));
          hoaCalculator_.calculate(earMetadata, hoaGains);

          auto routing = static_cast<std::size_t>(item.routing());
          for (int i(0); i < earMetadata.degrees.size(); i++) {
            direct_[routing + i] = hoaGains[i];
          }
        }
        if (item.has_bin_metadata()) {
          throw std::runtime_error(
              "received unsupported binaural type metadata");
        }
      }
    }

    // Used for setting the newItem flag next time around
    {
      std::lock_guard<std::mutex> lockA(allActiveIdsMutex_);
      allActiveIds.clear();
      for(const auto& item : storeToProcess->monitoring_items()) {
        allActiveIds.push_back(item.connection_id());
      }
    }

    storeToProcess.reset();

  });

  future.get();
}

namespace {
Eigen::MatrixXf toEigenMat(std::vector<std::vector<float>>& vec) {
  auto outputCount = vec.size();
  auto inputCount = static_cast<int>(vec[0].size());
  Eigen::MatrixXf mat(inputCount, outputCount);
  for (std::size_t i = 0; i < outputCount; i++) {
    mat.col(static_cast<int>(i)) =
        Eigen::VectorXf::Map(vec[i].data(), inputCount);
  }
  return mat;
}
}  // namespace

Eigen::MatrixXf SceneGainsCalculator::directGains() {
  return toEigenMat(direct_);
}

Eigen::MatrixXf SceneGainsCalculator::diffuseGains() {
  return toEigenMat(diffuse_);
}

void SceneGainsCalculator::clear() {
  for (auto& gainVec : direct_) {
    std::fill(gainVec.begin(), gainVec.end(), 0.0f);
  }
  for (auto& gainVec : diffuse_) {
    std::fill(gainVec.begin(), gainVec.end(), 0.0f);
  }
}

std::vector<communication::ConnectionId> SceneGainsCalculator::removedIds(
    const proto::SceneStore& store) const {
  std::vector<communication::ConnectionId> removedIds;
  for (auto& entry : routingCache_) {
    auto id = entry.first;
    auto it = std::find_if(
        store.monitoring_items().begin(), store.monitoring_items().end(),
        [id](auto& item) {
          return communication::ConnectionId{item.connection_id()} == id;
        });
    if (it == store.monitoring_items().end()) {
      removedIds.push_back(entry.first);
    }
  }
  return removedIds;
}

std::vector<Routing> SceneGainsCalculator::updateRoutingCache(
    const proto::SceneStore& store) {
  std::vector<Routing> changedRouting;
  for (auto& item : store.monitoring_items()) {
    if (item.changed()) {
      int size = 1;
      if (item.has_ds_metadata()) {
        size = item.ds_metadata().speakers().size();
      }
      if (item.has_hoa_metadata()) {
        std::shared_ptr<AdmCommonDefinitionHelper::PackFormatData> pfData;
        {
          std::lock_guard<std::mutex> lock(commonDefinitionHelperMutex_);
          auto temp = item.hoa_metadata().packformatidvalue();
          pfData = commonDefinitionHelper.getPackFormatData(4, temp);
        }
        if (pfData) {
          auto cfData = pfData->relatedChannelFormats;
          size = cfData.size();
        }
      }
      Routing newRouting{item.routing(), size};
      auto it = routingCache_.find(item.connection_id());
      if (it != routingCache_.end() && (it->second.track != newRouting.track ||
                                        it->second.size != newRouting.size)) {
        changedRouting.push_back(it->second);
      }
      if (newRouting.track >= 0) {
        routingCache_[item.connection_id()] = newRouting;
      } else {
        routingCache_.erase(item.connection_id());
      }
    }
  }
  return changedRouting;
}

void SceneGainsCalculator::resize(ear::Layout& outputLayout,
                                  std::size_t inputChannelCount)
{
  {
    std::lock_guard<std::mutex> lock(gainVectorsMutex_);
    direct_.resize(inputChannelCount);
    diffuse_.resize(inputChannelCount);
    auto outputChannelCount = outputLayout.channels().size();
    for(auto& gainVec : direct_) {
      gainVec.resize(outputChannelCount, 0.0f);
    }
    for(auto& gainVec : diffuse_) {
      gainVec.resize(outputChannelCount, 0.0f);
    }
  }
  {
    std::lock_guard<std::mutex> lock(allActiveIdsMutex_);
    allActiveIds.reserve(inputChannelCount);
  }
}

}  // namespace plugin
}  // namespace ear
