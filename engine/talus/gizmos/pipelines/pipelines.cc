#include "pipelines.ih"

namespace talus::gizmos {

std::vector<talus::gpu::PipelineDescription> pipelines() {
  return {Grid::describe(), Outline::describe()};
}

}
