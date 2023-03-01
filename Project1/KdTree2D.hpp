#ifndef ZHI_KDTREE_H_
#define ZHI_KDTREE_H_

#include <vector>
#include <numeric>
#include <algorithm>
#include <exception>
#include <functional>

#include "Vector3.hpp"


struct Node {
	vector3 point;
	int id;
	Node* left;
	Node* right;

	Node(vector3 point, int setId) :
		point(point),
		id(setId),
		left(nullptr),
		right(nullptr) {}
};

struct KdTree2D {
public: 
	KdTree2D() : root(nullptr) {}
	~KdTree2D() { clear(); }

	void insertHelper(Node** node, size_t depth, vector3 point, int id) {
		if (*node == nullptr) {
			*node = new Node(point, id);
		}
		else {
			size_t cd = depth % 2;
			if (point[cd] < ((*node)->point[cd])) {
				insertHelper(&((*node)->left), depth + 1, point, id);
			}
			else {
				insertHelper(&((*node)->right), depth + 1, point, id);
			}
		}
	}

	void insert(vector3 point, int id) {
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(vector3 target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {
		if (node != nullptr) {
			if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol))) {
				float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]));
				if (distance <= distanceTol) {
					ids.push_back(node->id);
				}
			}

			if ((target[depth % 2] - distanceTol) < node->point[depth % 2]) {
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			}
			if ((target[depth % 2] + distanceTol) > node->point[depth % 2]) {
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}
		}
	}

	std::vector<int> search(vector3 target, float distanceTol) {
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}

private:
	void clearHelper(Node** node) {
		if (node == nullptr)
			return;
		if ((*node)->left)
			clearHelper(&(*node)->left);
		if ((*node)->right)
			clearHelper(&(*node)->right);
		delete* node;
	}

	void clear() {
		clearHelper(&root);
		root = nullptr;
	}

private:
	Node* root;
};

#endif









