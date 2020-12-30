/* \ original author Aaron Brown (Udacity KD-Tree Quiz)*/
/* additional implementation by Christopher Anton*/

#ifndef KDTREE_H_
#define KDTREE_H_
#include <cmath>

// Structure to represent node of kd tree
template<typename PointT>
struct KdTree 
{
    struct Node 
    {
        PointT point;
        int id;
        Node *left;
        Node *right;

        Node(PointT arr, int setId)
                : point(arr), id(setId), left(NULL), right(NULL) {}
    };

    Node* root;

    KdTree()
            : root(NULL) {}

    //Helps in forming a new node as the root
    void insertHelper(Node **node, int depth, PointT point, int id) 
    {
    
        // If Tree is empty
        if (*node == NULL)
            *node = new Node(point, id);
        else 
        {            
            //calculate current dimension 
            uint cd = depth % 3;
            if (cd == 0) 
                if (point.x < ((*node)->point.x))
                    insertHelper(&((*node)->left), depth + 1, point, id);
                else
                    insertHelper(&((*node)->right), depth + 1, point, id);
            if (cd == 1) 
                if (point.y < ((*node)->point.y))
                    insertHelper(&((*node)->left), depth + 1, point, id);
                else
                    insertHelper(&((*node)->right), depth + 1, point, id);
            if (cd == 2) 
                if (point.z < ((*node)->point.z))
                    insertHelper(&((*node)->left), depth + 1, point, id);
                else
                    insertHelper(&((*node)->right), depth + 1, point, id);
        }
    }

    void insert(PointT point, int id) 
    {
      insertHelper(&root, 0, point, id);
    }

    void searchHelper(PointT target, Node *node, int depth, float distanceTol, std::vector<int> &ids) 
    {
        if (node != NULL) 
        {

            bool xpt = (node->point.x >= (target.x - distanceTol)) && (node->point.x <= (target.x + distanceTol));
            bool ypt = (node->point.y >= (target.y - distanceTol)) && (node->point.y <= (target.y + distanceTol));
            bool zpt = (node->point.z >= (target.z - distanceTol)) && (node->point.z <= (target.z + distanceTol));

            if (xpt && ypt && zpt)
            {
                float distance = sqrt(pow((node->point.x - target.x),2) + pow((node->point.y - target.y),2) + pow((node->point.z - target.z),2));
                if (distance <= distanceTol)
                    ids.push_back(node->id);
            }
            
            //check across boundary
            int currentdim = depth % 3;

            if (currentdim == 0) {
                if ((target.x - distanceTol) < node->point.x)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.x + distanceTol) > node->point.x)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
            if (currentdim == 1) {
                if ((target.y - distanceTol) < node->point.y)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.y + distanceTol) > node->point.y)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
            if (currentdim == 2) {
                if ((target.z - distanceTol) < node->point.z)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.z + distanceTol) > node->point.z)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }

        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(PointT target, float distanceTol) 
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }


};

#endif /* KDTREE_H_ */