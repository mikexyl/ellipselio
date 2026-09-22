#include <iostream>
#include <array>
#include <numeric>
#include "ioctree/ioctree.h"

struct Point { float x,y,z; };
struct TestTree : iOctree::Octree {
  size_t capacity() const { return octant_max; }
  size_t allocated() const { return octant_num; }
  std::vector<float*> pointers() {
    std::vector<const iOctree::Octant*> nodes;
    GetLeafNodes(root_,nodes);
    std::vector<float*> result;
    for (auto* node:nodes) for (auto* point:node->points) result.push_back(point);
    return result;
  }
};

int main() {
  TestTree tree;
  tree.SetMaxNewPoints(64);
  tree.SetMaxOctants(2); // Deliberately force many allocations and leaf splits.
  tree.SetBucketSize(2);
  tree.SetMinExtent(.001f);
  std::vector<Point> expected;
  for (int batch=0;batch<32;++batch) {
    auto old=tree.pointers();
    std::vector<std::array<float,4>> values;
    for(auto* p:old)values.push_back({p[0],p[1],p[2],p[3]});
    std::vector<Point> points;
    for(int j=0;j<32;++j) {
      const int i=batch*32+j;
      points.push_back({float((i%31)*.41+.001*(i%3)),float((i/31)*.37),float((i%7)*.29)});
    }
    std::vector<int> filters(points.size()),added,ids;
    std::iota(filters.begin(),filters.end(),0);
    tree.Update(points,int(points.size()),filters,added,ids,true);
    assert(added.size()==points.size());
    for(size_t k=0;k<ids.size();++k) {
      if(size_t(ids[k])>=expected.size())expected.resize(ids[k]+1);
      expected[ids[k]]=points[added[k]];
    }
    // Growing storage must not relocate or overwrite existing leaf buffers.
    for(size_t k=0;k<old.size();++k)
      for(int d=0;d<4;++d)assert(old[k][d]==values[k][d]);
    // Independently check query indices and distances against inserted geometry.
    for(size_t i=0;i<expected.size();i+=7) {
      std::vector<size_t> neighbors;std::vector<float> distance;
      Point query=expected[i];query.x+=.001f; // Upstream intentionally excludes zero-distance self matches.
      tree.RadiusNeighbors(query,.02f,neighbors,distance);
      if(neighbors.size()!=1 || neighbors[0]!=i) {
        std::cerr<<"batch="<<batch<<" query="<<i<<" xyz="<<expected[i].x<<","<<expected[i].y<<","<<expected[i].z<<" result=";
        for(auto n:neighbors)std::cerr<<n<<",";
        std::cerr<<"\n";
      }
      assert(neighbors.size()==1 && neighbors[0]==int(i));
      const float dx=query.x-expected[i].x;
      assert(distance.size()==1 && std::abs(distance[0]-dx*dx)<1e-10f);
    }
    assert(tree.capacity()>=tree.allocated());
  }
  assert(tree.allocated()>100 && tree.capacity()>2);
  tree.clear();
  tree.SetMaxOctants(1);
  std::vector<Point> singleton={{1,2,3}};
  std::vector<int> filters={0},added,ids,neighbors;
  tree.Initialize(singleton,1,filters,added,ids,true);
  assert(tree.allocated()==1 && tree.capacity()==1);
  tree.RadiusNeighbors(Point{1.001f,2,3},.1f,neighbors);
  assert(neighbors==std::vector<int>{0});
  std::cout<<"Octree growth, stable pointers, brute-force query identities, clear/reuse, and exact boundary passed\n";
}
