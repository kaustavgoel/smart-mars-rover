#ifndef AVL_TREE_HPP
#define AVL_TREE_HPP

#include "Types.hpp"
#include <iostream>
#include <vector>

struct AVLNode {
  int key; // siteId
  SiteData data;
  int height;
  AVLNode *left;
  AVLNode *right;
  AVLNode(const SiteData &d)
      : key(d.siteId), data(d), height(1), left(nullptr), right(nullptr) {}
};

// balanced avl tree to store discovered sites
class AVLTree {
private:
  AVLNode *root;
  int nodeCount;
  int getHeight(AVLNode *node) const {
    return (node == nullptr) ? 0 : node->height;
  }
  int getBalanceFactor(AVLNode *node) const {
    if (node == nullptr)
      return 0;
    return getHeight(node->left) - getHeight(node->right);
  }

  void updateHeight(AVLNode *node) {
    if (node != nullptr) {
      node->height = 1 + findMax(getHeight(node->left), getHeight(node->right));
    }
  }

  // right rotation for LL case
  AVLNode *rotateRight(AVLNode *y) {
    AVLNode *x = y->left;
    AVLNode *t2 = x->right;
    x->right = y;
    y->left = t2;
    updateHeight(y);
    updateHeight(x);
    return x;
  }

  // left rotation for RR case
  AVLNode *rotateLeft(AVLNode *x) {
    AVLNode *y = x->right;
    AVLNode *t2 = y->left;

    y->left = x;
    x->right = t2;

    updateHeight(x);
    updateHeight(y);
    return y;
  }

  AVLNode *insertNode(AVLNode *node, const SiteData &data) {
    if (node == nullptr) {
      nodeCount++;
      return new AVLNode(data);
    }

    if (data.siteId < node->key) {
      node->left = insertNode(node->left, data);
    } else if (data.siteId > node->key) {
      node->right = insertNode(node->right, data);
    } else {
      // duplicate key updates data
      node->data = data;
      return node;
    }

    updateHeight(node);
    int balance = getBalanceFactor(node);

    // balance tree if skewed
    if (balance > 1 && data.siteId < node->left->key) {
      return rotateRight(node);
    }
    if (balance < -1 && data.siteId > node->right->key) {
      return rotateLeft(node);
    }
    if (balance > 1 && data.siteId > node->left->key) {
      node->left = rotateLeft(node->left);
      return rotateRight(node);
    }
    if (balance < -1 && data.siteId < node->right->key) {
      node->right = rotateRight(node->right);
      return rotateLeft(node);
    }

    return node;
  }

  AVLNode *searchNode(AVLNode *node, int key) const {
    if (node == nullptr || node->key == key)
      return node;
    if (key < node->key)
      return searchNode(node->left, key);
    return searchNode(node->right, key);
  }

  void inorder(AVLNode *node, std::vector<SiteData> &result) const {
    if (node != nullptr) {
      inorder(node->left, result);
      result.push_back(node->data);
      inorder(node->right, result);
    }
  }

  void destroyTree(AVLNode *node) {
    if (node != nullptr) {
      destroyTree(node->left);
      destroyTree(node->right);
      delete node;
    }
  }

public:
  AVLTree() : root(nullptr), nodeCount(0) {}

  ~AVLTree() { destroyTree(root); }

  void insert(const SiteData &site) { root = insertNode(root, site); }

  SiteData *search(int siteId) {
    AVLNode *res = searchNode(root, siteId);
    return (res != nullptr) ? &(res->data) : nullptr;
  }

  const SiteData *search(int siteId) const {
    AVLNode *res = searchNode(root, siteId);
    return (res != nullptr) ? &(res->data) : nullptr;
  }

  std::vector<SiteData> getAllSitesInOrder() const {
    std::vector<SiteData> result;
    inorder(root, result);
    return result;
  }

  int size() const { return nodeCount; }

  int getTreeHeight() const { return getHeight(root); }

  void clear() {
    destroyTree(root);
    root = nullptr;
    nodeCount = 0;
  }
};

#endif // AVL_TREE_HPP
