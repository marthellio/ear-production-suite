#pragma once
#include <vector>
#include <memory>
#include <boost/variant.hpp>
#include "scene_store.pb.h"
#include "communication/common_types.hpp"
#include <ear/ear.hpp>
#include <Eigen/Eigen>
#include <map>
#include <string>
#include <vector>
#include <optional>
#include "helper/common_definition_helper.h"

namespace ear {
namespace plugin {

struct GainHolder {
  Eigen::MatrixXf direct;
  Eigen::MatrixXf diffuse;
};

struct Routing {
  int track;
  int size;
};

class SceneGainsCalculator {
 public:
  SceneGainsCalculator(Layout outputLayout, int inputChannelCount);
  void update(proto::SceneStore store);
  Eigen::MatrixXf directGains();
  Eigen::MatrixXf diffuseGains();

 private:
  // returns ids that had been present at the last `update()` call,`
  // but are no longer in the given scene store
  std::vector<communication::ConnectionId> removedIds(
      const proto::SceneStore &store) const;

  // updates internal routing cache, adding new items and changing routing
  // values as needed
  //
  // @returns Map of item ids whose routing has changed with their
  // _previous_ routing values
  std::vector<Routing> updateRoutingCache(const proto::SceneStore &store);

  void resize(ear::Layout &ouputLayout, std::size_t inputChannelCount);
  void clear();

  std::mutex gainVectorsMutex_;
  std::vector<std::vector<float>> direct_;
  std::vector<std::vector<float>> diffuse_;

  std::mutex gainCalculatorsMutex_;
  ear::GainCalculatorObjects objectCalculator_;
  ear::GainCalculatorDirectSpeakers directSpeakersCalculator_;
  ear::GainCalculatorHOA hoaCalculator_;

  std::mutex routingCacheMutex_;
  std::map<communication::ConnectionId, Routing> routingCache_;

  std::mutex commonDefinitionHelperMutex_;
  AdmCommonDefinitionHelper commonDefinitionHelper{};

  std::mutex allActiveIdsMutex_;
  std::vector<std::string> allActiveIds;

  std::mutex storeToProcessMutex_;
  std::optional<proto::SceneStore> storeToProcess;

};

}  // namespace plugin
}  // namespace ear
