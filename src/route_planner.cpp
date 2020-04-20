#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;


    RoutePlanner::start_node = &m_Model.FindClosestNode(start_x, start_y);
    RoutePlanner::end_node = &m_Model.FindClosestNode(end_x, end_y);

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return (*node).distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    (*current_node).FindNeighbors();
    for(RouteModel::Node *ptr : (*current_node).neighbors){
        (*ptr).parent = current_node;
        (*ptr).h_value = RoutePlanner::CalculateHValue(ptr);
        (*ptr).g_value = (*current_node).g_value + (*ptr).distance(*current_node);
        open_list.push_back(ptr);
        (*ptr).visited = true;
    }

}

bool CompareNodes(RouteModel::Node *node1, RouteModel::Node *node2){
    float f1 = node1->g_value + node1->h_value;
    float f2 = node2->g_value + node2->h_value;
    return f1 > f2;
}
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), CompareNodes);
    RouteModel::Node *ptr = open_list.back();
    open_list.pop_back();
    return ptr;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while(start_node != current_node){
        distance += (*current_node).distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
  	current_node->visited = true;
    while(current_node != end_node){
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    RoutePlanner::m_Model.path = ConstructFinalPath(current_node);
}