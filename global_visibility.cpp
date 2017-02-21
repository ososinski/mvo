/*
 */

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <vector>

using namespace std;
using namespace octomap;

void visibility(const octomap::OcTree& oc,const octomap::OcTreeVolume& v,const point3d origin, double* values);

double vis_value(double distance, double angle);

const double MIN_TLS_RANGE = 0.5;
const double MAX_TLS_RANGE = 75;
const double PARTIAL_VISIBILITY = 0.3;

int main(int argc, char** argv) {

  OcTree octree("input.bt");
  //vector <point3d> viewpoints();

  // insert some measurements of occupied cells
  point3d result;
  OcTreeVolume voxel;

  std::vector<point3d> the_chosen; 
  // ignore the outer bounds
the_chosen.push_back(point3d(7,6,0));
the_chosen.push_back(point3d(13,5,0));
the_chosen.push_back(point3d(1,2,0));
the_chosen.push_back(point3d(10,14,0));
the_chosen.push_back(point3d(17,5,0));
the_chosen.push_back(point3d(5,-3,0));
the_chosen.push_back(point3d(3,7,0));
the_chosen.push_back(point3d(13,9,0));
the_chosen.push_back(point3d(7,1,0));
the_chosen.push_back(point3d(8,11,0));
the_chosen.push_back(point3d(11,2,0));

  double global_visibility = 0;
  double global_points = 0;
  double face_visibility[6] = {0,0,0,0,0,0};

 

  //do{
    global_points = 0;
    for(OcTree::tree_iterator it = octree.begin_tree(octree.getTreeDepth()),
            end=octree.end_tree(); it!= end; ++it) {

      if (it.isLeaf()) { // voxels for leaf nodes
        voxel = OcTreeVolume(it.getCoordinate(), it.getSize());
        if (octree.isNodeOccupied(*it)){ // occupied voxels
          if (octree.isNodeAtThreshold(*it)) {
              global_points +=6;
              for(int k = 0;k<6;k++){
                 face_visibility[k]=0; //clean the array
              }
              for(vector<point3d>::iterator itr = the_chosen.begin(); itr != the_chosen.end(); ++itr){
                 visibility(octree, voxel,(*itr), face_visibility);              
              }
              //integrate results into a global measure
              for(int l = 0;l<6;l++){
                  if (face_visibility[l]>0){
                     global_visibility+=face_visibility[l];
                  }
              }
          }
        }//else{cout<<"not occupied?"<<endl;}
      }
    }

  //std::cout

 std::cout <<"% global_visibility("<<global_visibility/global_points<<")"<< endl;  

 //}while(the_chosen.size()<15);

}


double vis_value(double distance, double angle){
   double vis = 0.5;
   if((distance>MIN_TLS_RANGE)&&(distance<MAX_TLS_RANGE)) vis+=0.3-(0.3*(distance/MAX_TLS_RANGE));
   //if(angle
   //cout << angle << endl;
   vis +=(fabs(angle - 1.57)/1.57)*0.2;
   return vis;
}

void visibility(const octomap::OcTree& oc,const octomap::OcTreeVolume& v, const point3d origin, double* values) {
    //viewpoints.push_back(
    point3d end;
        // colour as the distance from the viewpoint zero to the point



// epsilon to be substracted from cube size so that neighboring planes don't overlap
    // seems to introduce strange artifacts nevertheless...
    double eps = 1e-5;
    
    point3d p = v.first;
    //std::cout << "point("<<p.x()<<","<<p.y()<<","<<p.z()<<")"<<std::endl; 
    double half_cube_size = float((v.second*0.5)-eps);


/*

b 1,5,4,0
f 3,7,6,2
t 1,0,2,3
d 5,4,6,7
l 1,3,7,4
r 0,2,6,4

0 rbt( 1, 1, 1)
1 lbt( 1, 1,-1)
2 rft( 1,-1, 1)
3 lft( 1,-1,-1)
4 rbd(-1, 1, 1)
5 lbd(-1, 1,-1)
6 rfd(-1,-1, 1)
7 lfd(-1,-1,-1)
*/
bool rbt=false,lbt=false,rft=false,lft=false,rbd=false,lbd=false,rfd=false,lfd=false;

    //corner points
    point3d p_rbt( half_cube_size+p.x(), half_cube_size+p.y(), half_cube_size+p.z());
    point3d p_lbt( half_cube_size+p.x(), half_cube_size+p.y(),-half_cube_size+p.z());
    point3d p_rft( half_cube_size+p.x(),-half_cube_size+p.y(), half_cube_size+p.z());
    point3d p_lft( half_cube_size+p.x(),-half_cube_size+p.y(),-half_cube_size+p.z());
    point3d p_rbd(-half_cube_size+p.x(), half_cube_size+p.y(), half_cube_size+p.z());
    point3d p_lbd(-half_cube_size+p.x(), half_cube_size+p.y(),-half_cube_size+p.z());
    point3d p_rfd(-half_cube_size+p.x(),-half_cube_size+p.y(), half_cube_size+p.z());
    point3d p_lfd(-half_cube_size+p.x(),-half_cube_size+p.y(),-half_cube_size+p.z());


    // face points
    point3d p_b(p.x(), half_cube_size+p.y(),p.z());
    point3d p_f(p.x(),-half_cube_size+p.y(),p.z());
    point3d p_t( half_cube_size+p.x(), p.y(), p.z());
    point3d p_d(-half_cube_size+p.x(), p.y(), p.z());
    point3d p_l(p.x(), p.y(), -half_cube_size+p.z());
    point3d p_r(p.x(), p.y(),  half_cube_size+p.z());

    /* check if the ray hits a cube , exact cube || close enough cube */
    if(oc.castRay(origin,p_rbt-origin,end,true)){ rbt=(end==p||end.distance(p_rbt)<v.second); }
    if(oc.castRay(origin,p_lbt-origin,end,true)){ lbt=(end==p||end.distance(p_lbt)<v.second); }
    if(oc.castRay(origin,p_rft-origin,end,true)){ rft=(end==p||end.distance(p_rft)<v.second); }
    if(oc.castRay(origin,p_lft-origin,end,true)){ lft=(end==p)||end.distance(p_lft)<v.second; }
    if(oc.castRay(origin,p_rbd-origin,end,true)){ rbd=(end==p)||end.distance(p_rbd)<v.second; }
    if(oc.castRay(origin,p_lbd-origin,end,true)){ lbd=(end==p)||end.distance(p_lbd)<v.second; }
    if(oc.castRay(origin,p_rfd-origin,end,true)){ rfd=(end==p)||end.distance(p_rfd)<v.second; }
    if(oc.castRay(origin,p_lfd-origin,end,true)){ lfd=(end==p||end.distance(p_lfd)<v.second); }      

    /* viewpoints loop  end*/


//

    double vis;
    point3d vp;

    //back
    if(rbt&&lbt&&rbd&&lbd){
       vp = origin - p_b; // calculate face normal
       vis = vis_value(p_b.distance(origin),vp.angleTo(p_b-p));
       if(values[0]<vis) values[0]=vis;
    }
    else if(rbt||lbt||rbd||lbd){if(values[0]<=0) values[0]=PARTIAL_VISIBILITY;}


    //front
    if(rft&&lft&&rfd&&lfd){
       vp = origin - p_f;
       vis = vis_value(p_f.distance(origin),vp.angleTo(p_f-p));
       if(values[1]<vis) values[1]=vis;
    }
    else if(rft||lft||rfd||lfd){if(values[1]<=0) values[1]=PARTIAL_VISIBILITY;}


    //top
    if(rft&&lft&&rbt&&lbt){
       vp = origin - p_t;
       vis = vis_value(p_t.distance(origin),vp.angleTo(p_t-p));
       if(values[2]<vis) values[2]=vis;
    }
    else if(rft||lft||rbt||lbt){if(values[2]<=0) values[2]=PARTIAL_VISIBILITY;}


    //down (bottom)
    if(rfd&&lfd&&rbd&&lbd){
       vp = origin - p_d;
       vis = vis_value(p_d.distance(origin),vp.angleTo(p_d-p));
       if(values[3]<vis) values[3]=vis;
    }
    else if(rfd||lfd||rbd||lbd){if(values[3]<=0) values[3]=PARTIAL_VISIBILITY;}


    //left
    if(lft&&lfd&&lbt&&lbd){
       vp = origin - p_l;
       vis = vis_value(p_l.distance(origin),vp.angleTo(p_l-p));
       if(values[4]<vis) values[4]=vis;
    }
    else if(lft||lfd||lbd||lbd){if(values[4]<=0) values[4]=PARTIAL_VISIBILITY;}


    //right
    if(rft&&rfd&&rbt&&rbd){
       vp = origin - p_r;
       vis = vis_value(p_r.distance(origin),vp.angleTo(p_r-p));
       if(values[5]<vis) values[5]=vis;
    }
    else if(rft||rfd||rbd||rbd){if(values[5]<=0) values[5]=PARTIAL_VISIBILITY;}
  }
