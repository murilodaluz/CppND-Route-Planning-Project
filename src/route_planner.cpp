#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    //Find pointers to closest nodes to start/end position
    //Note: coordinates are changed to percent
    start_node = &m_Model.FindClosestNode((start_x * 0.01), (start_y * 0.01));
    end_node   = &m_Model.FindClosestNode((end_x * 0.01), (end_y * 0.01));
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node){
    //empty path and 0 distance
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    //Add current node to begining of the path and update total distance
    while (current_node->parent != nullptr)
    {
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }

    path_found.push_back(*current_node);
    //Apply scale to distance
    distance *= m_Model.MetricScale();
    return path_found;
}

float RoutePlanner::CalculateHValue(const RouteModel::Node *node){
    node->distance(*end_node);
}

RouteModel::Node * RoutePlanner::NextNode()
{
    //sort the vector of Nodes
    std::sort(open_list.begin(), open_list.end(), [](const auto &_1st, const auto &_2nd){ 
        return _1st->h_value + _1st->g_value < _2nd->h_value + _2nd->g_value; 
    });

    //Get smallest F value node, and pop-it from list
    RouteModel::Node *lowest_node = open_list.front();
    open_list.erase(open_list.begin());
    return lowest_node;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node){       
    //Expand the current node (add all unvisited neighbors to the open list);
    current_node->FindNeighbors();

    for(RouteModel::Node *neighbor : current_node->neighbors){
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor);
        
        // Add the neighbor to the open list;
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}

void RoutePlanner::AStarSearch(){
    // Initializa open_list with starting node;
    start_node->visited = true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;
    
    // Expand nodes util you reach the goal. Use heuristic to prioritize what node to open first;
    while (open_list.size() > 0){
        // Select the best node to explore next;
        current_node = NextNode();
        // Check if the node selected is the goal;
        if(current_node->distance(*end_node) == 0){
            // Set the model path variable with the path found;
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        AddNeighbors(current_node);
    }
    
}