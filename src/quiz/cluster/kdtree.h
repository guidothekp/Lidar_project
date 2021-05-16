/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <string>

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
        :	point(arr), id(setId), left(NULL), right(NULL)
    {}

    std::string toString() {
        auto message = "{id: " + std::to_string(id);
        message += "\nleft : " + (left == NULL? "NULL": left->toString());
        message += "\nright: " + (right == NULL? "NULL": right->toString());
        message += "}\n";
        return message;
    }
};

struct KdTree
{
    Node* root;

    KdTree()
        : root(NULL)
    {}


    //inorder listing
    void list(const Node * root) {
        if (root == NULL) {
            return;
        }
        list(root->left);
        std::cout << "(";
        for (int i = 0; i < root->point.size(); i ++) {
            std::cout << root->point[i];
            if (i != root->point.size() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << ")";
        list(root->right);
    }

    void list() {
        list(root);
    }

    void insert(Node* & root, std::vector<float> & point, int id, int cutting_dimension, const uint dimension = 2) {
        if (root == NULL) {
            root = new Node(point, id);
            return;
        }
        //int index = count % 2;
        bool left = point[cutting_dimension] < root->point[cutting_dimension];
        if (left) {
            insert(root->left, point, id, (cutting_dimension + 1) % dimension);
        } else {
            insert(root->right, point, id, (cutting_dimension + 1) % dimension);
        }
    }

    void insert(std::vector<float> & point, int id) {
        insert(root, point, id, 0); 
    }

    void insertNR(std::vector<float> & point, int id)
    {
        auto to_string = [&](std::vector<float> c) -> std::string {
            std::string message = "(";
            for (int i = 0; i < c.size(); i ++) {
                message += std::to_string(c[i]);
                if (i != c.size() - 1) {
                    message += ", ";
                }
            }
            message += ")";
            return message;
        };
        std::cout << "-------------------\n";
        std::cout << "insert called for id=" << id << std::endl;;
        //print(point);
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root 
        if (root == NULL) {
            std::cout << "adding as root " << to_string(point) << std::endl; 
            root = new Node(point, id);
            return;
        }
        auto current = root;
        int count = 0;
        while(true) {
            int index = count % 2; //0 => x, 1 => y
            bool left = point[index] < current->point[index];
            //std::cout << "going " << (left? "left" : "right") << std::endl;
            //std::cout << "comparing: " << std::to_string(point[index]) << " with " << std::to_string(current->point[index]) << std::endl;
            auto next = left? current->left : current->right;
            if (next == NULL) {
                std::cout << "adding " << to_string(point) << " as " << (left ? "left" : "right") << " child of "; 
                std::cout << to_string(current->point) << std::endl;
                next = new Node(point, id);
                //determine if this is the left or right child
                if (left) {
                    current->left = next;
                } else {
                    current->right = next;
                }
                break;
            } else {
                current = next;
            }
            ++ count;
        }
    }

    bool isWithInBounds(std::vector<float> & target, float width, Node* node) {
        for (int i = 0; i < target.size(); i ++ ) {
            if (target[i] - width > node->point[i] || target[i] + width < node->point[i]) 
                return false;
        }
        return true;
    }

    bool isWithInCircleAt(std::vector<float> & target, float radius, Node* node) {
        float distance = 0;
        for (int i = 0; i < target.size(); i ++) {
            float d = target[i] - node->point[i];
            distance += d * d;
        }
        return sqrt(distance) < radius;
    }


    void searchHelper(std::vector<float> & target, Node* start, int depth, float distanceTol, std::vector<int> & ids) {
        if (!start) {
            return;
        }
        int cd = depth % target.size();
        if (isWithInBounds(target, distanceTol, start) && isWithInCircleAt(target, distanceTol, start)) {
                ids.push_back(start->id);
        }
        if (target[cd] - distanceTol < start->point[cd]) {
            searchHelper(target, start->left, depth + 1, distanceTol, ids);
        }
        if (target[cd] + distanceTol > start->point[cd]) {
            searchHelper(target, start->right, depth + 1, distanceTol, ids);
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids); 
        return ids;
    }

    std::string toString() {
        auto message = (root == NULL ? "NULL" : root->toString());
        return message;
    }
};




