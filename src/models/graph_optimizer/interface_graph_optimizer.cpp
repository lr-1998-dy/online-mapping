/*
 * @Description: 
 * @Author: Li Rui
 * @Date: 2022-03-01 18:35:19
 */

#include "lidar_localization/models/graph_optimizer/interface_graph_optimizer.hpp"

namespace lidar_localization {
void InterfaceGraphOptimizer::SetMaxIterationsNum(int max_iterations_num) {
    max_iterations_num_ = max_iterations_num;
}
}