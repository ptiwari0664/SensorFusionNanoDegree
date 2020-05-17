/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef KDTREE_H
#define KDTREE_H

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {
    }
};

struct KdTree {
  private:
    static constexpr int X{ 0 }, Y{ 1 }, Z{ 2 };

    void recursiveInsert(Node **node, const std::vector<float> point, const int id, const int coord) {

        if (*node == nullptr) {
            *node = new Node(point, id);
            return;
        }

        const bool goLeft{ point.at(coord) < (*node)->point.at(coord) };

        recursiveInsert((goLeft ? &(*node)->left : &(*node)->right), point, id, ((coord + 1) % 3));
    }

    bool withinBox(const std::vector<float> target, const Node *node, const float tolerance) {

        const float leftX{ target.at(X) - tolerance };
        const float rightX{ target.at(X) + tolerance };

        const bool withinXbounds{ node->point.at(X) >= leftX && node->point.at(X) <= rightX };

        const float lowerY{ target.at(Y) - tolerance };
        const float upperY{ target.at(Y) + tolerance };

        const bool withinYbounds{ node->point.at(Y) >= lowerY && node->point.at(Y) <= upperY };

        const float lowerZ{ target.at(Z) - tolerance };
        const float upperZ{ target.at(Z) + tolerance };

        const bool withinZbounds{ node->point.at(Z) >= lowerZ && node->point.at(Z) <= upperZ };

        return (withinXbounds && withinYbounds && withinZbounds);
    }

    float withinRange(const std::vector<float> p1, const std::vector<float> p2, const float tolerance) {
        const float xComponent{ p1.at(X) - p2.at(X) };
        const float yComponent{ p1.at(Y) - p2.at(Y) };
        const float zComponent{ p1.at(Z) - p2.at(Z) };

        const float distance{ std::sqrt((xComponent * xComponent) + (yComponent * yComponent) +
                                        (zComponent * zComponent)) };

        return (distance <= tolerance);
    }

    void recursiveSearch(Node *node, const std::vector<float> &target, std::vector<int> &ids,
                         const float tolerance, const int coord) {

        if (node == nullptr) { return; }

        if (withinBox(target, node, tolerance) && withinRange(node->point, target, tolerance))
            ids.push_back(node->id);

        if ((target.at(coord) - tolerance) < node->point.at(coord))
            recursiveSearch(node->left, target, ids, tolerance, ((coord + 1) % 3));

        if ((target.at(coord) + tolerance) > node->point.at(coord))
            recursiveSearch(node->right, target, ids, tolerance, ((coord + 1) % 3));
    }

  public:
    Node *root;

    KdTree() : root(NULL) {
    }

    void insert(const std::vector<float> point, int id) {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root

        recursiveInsert(&root, point, id, X);
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const std::vector<float> target, const float distanceTol) {
        Node *node{ root };
        std::vector<int> ids;

        recursiveSearch(root, target, ids, distanceTol, X);

        return ids;
    }
};

#endif
