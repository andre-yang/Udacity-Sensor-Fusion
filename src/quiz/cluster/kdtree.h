/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if (this->root == NULL)
		{
			root = new Node(point, id);
		} else {
			Node *node (new Node(point, id));
			insertKd(root, node, 0);
		}
	}

	void insertKd(Node *&curr_node, Node *&node, int depth) {
		int axis_compare = depth % 2;

		if (node->point[axis_compare] < curr_node->point[axis_compare]) {
			if (curr_node->left == NULL) {
				curr_node->left = node;
			} else {
				insertKd(curr_node->left, node, depth + 1);
			}
		} else {
			if (curr_node->right == NULL) {
				curr_node->right = node;
			} else {
				insertKd(curr_node->right, node, depth + 1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_recur(ids, root, target, distanceTol, 0);
		return ids;
	}
	
	void search_recur(std::vector<int> &ids, Node *&curr_node, std::vector<float> &target, float distanceTol, int depth) {
		
		if ( (curr_node->point[0] <= (target[0] + distanceTol) ) &&
			 (curr_node->point[0] >= (target[0] - distanceTol) ) &&
			 (curr_node->point[1] <= (target[1] + distanceTol) ) &&
			 (curr_node->point[1] >= (target[1] - distanceTol) ) )
		{
			if ( pow(pow(curr_node->point[0]-target[0],2) + pow(curr_node->point[1]-target[1],2), 0.5) <= distanceTol ) {
				ids.push_back(curr_node->id);
			}
		}

		// search left if the bounding box is within the left side of the splitting line, and vice versa
		int axis_compare = depth % 2;
		if ((target[axis_compare] - distanceTol) < curr_node->point[axis_compare]) {
			if (curr_node->left != NULL) {
				search_recur(ids, curr_node->left, target, distanceTol, depth+1);	
			}
		} 
		
		if ((target[axis_compare] + distanceTol) > curr_node->point[axis_compare]) {
			if (curr_node->right != NULL) {
				search_recur(ids, curr_node->right, target, distanceTol, depth+1);	
			}
		}
	}

};




