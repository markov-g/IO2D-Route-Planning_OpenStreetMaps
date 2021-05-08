#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
}


// Expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *neighbor: current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = this->CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);

        this->open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    // Sort open list according to sum of current distance + heuristic
   sort(this->open_list.begin(), this->open_list.end(), [](const auto &_1st, const auto &_2nd){
       return _1st->h_value + _1st->g_value < _2nd->h_value + _2nd->g_value;
   });
   RouteModel::Node *next_node = this->open_list.front();
   this->open_list.erase(this->open_list.begin());
   
   return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr) {
        distance += current_node->distance(*current_node->parent);
        path_found.emplace(path_found.begin(), *current_node);
        current_node = current_node->parent;
    }
    path_found.emplace(path_found.begin(), *current_node); // add also the final node

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    RoutePlanner::start_node->visited = true;
  	RoutePlanner::open_list.emplace_back(RoutePlanner::start_node);
  
  	while (!RoutePlanner::open_list.empty()) {
        current_node = RoutePlanner::NextNode();
        if (current_node->distance(*RoutePlanner::end_node) == 0) {
            RoutePlanner::m_Model.path = RoutePlanner::ConstructFinalPath(current_node);
            return;
        }
      
        RoutePlanner::AddNeighbors(current_node);
    }
}