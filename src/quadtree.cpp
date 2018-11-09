#include "quadtree.h"

QuadTree::QuadTree(double x, double y,  double w, double h){
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;

    if (freeSpace(x, y, w, h)){
        divided = false;
    }
    else{
        divided = true;
        nw = QuadTree(x, y, w/2, h/2);
        ne = QuadTree(x + w/2, y, w/2, h/2);
        sw = QuadTree(x, y + h/2, w/2, h/2);
        se = QuadTree(x + w/2, y + h/2, w/2, h/2);
    }
}



bool freeSpace(double x, double y, double w, double h){
    
}
