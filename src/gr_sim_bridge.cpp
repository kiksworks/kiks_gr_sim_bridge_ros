// Copyright 2023 KIKS
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


#include "rclcpp/context.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "kiks_gr_sim_bridge/sender_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto sender_node = std::make_shared<kiks::gr_sim_bridge::SenderNode>();
  exec.add_node(sender_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
