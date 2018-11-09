class QuadTree
{
public:
    quadtree(double x, double y,  double w, double h);
    ~quadtree();

    bool isDivided();
    void divide();


private:
    QuadTree nw, ne, sw, se;
    double x, y, w, h;
    bool divided;
    bool free;



    
};