#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &(model.FindClosestNode(start_x, start_y));
    this->end_node = &(model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return node->distance(*(this->end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();
    for (RouteModel::Node *node : current_node->neighbors)
    {
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + node->distance(*current_node);
        node->visited = true;
        open_list.emplace_back(node);
    }
}

RouteModel::Node *RoutePlanner::NextNode()
{
    std::sort(open_list.begin(), open_list.end(), [](const auto &node_1, const auto &node_2)
              { return (node_1->h_value + node_1->g_value) >= (node_2->h_value + node_2->g_value); });

    RouteModel::Node *node = open_list.back();
    open_list.pop_back();
    return node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    path_found.emplace_back(*(current_node));
    while (current_node->parent != nullptr)
    {
        distance += current_node->distance(*(current_node->parent));
        path_found.emplace_back(*(current_node->parent));
        current_node = current_node->parent;
    }
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = nullptr;
    open_list.emplace_back(this->start_node);
    this->start_node->visited = true;
    while (open_list.size() > 0)
    {
        current_node = NextNode();
        if (current_node->distance(*this->end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);
    }
}