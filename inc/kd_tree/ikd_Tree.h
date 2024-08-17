#pragma once
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <chrono>
#include <pcl/point_types.h>
#include <pthread.h>
#include <queue>
#include <stdio.h>
#include <time.h>

#define EPSS 1e-6
#define Minimal_Unbalanced_Tree_Size 10
#define Multi_Thread_Rebuild_Point_Num 1500
#define DOWNSAMPLE_SWITCH true
#define ForceRebuildPercentage 0.2
#define Q_LEN 1000000

using namespace std;

typedef pcl::PointXYZINormal PointType;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;

const PointType ZeroP;

struct KDTreeNode
{
    PointType point;
    int division_axis;  
    int TreeSize = 1;
    int invalid_point_num = 0;
    int down_del_num = 0;
    bool point_deleted = false;
    bool tree_deleted = false; 
    bool point_downsample_deleted = false;
    bool tree_downsample_deleted = false;
    bool need_push_down_to_left = false;
    bool need_push_down_to_right = false;
    bool working_flag = false;
    pthread_mutex_t push_down_mutex_lock;
    float node_range_x[2], node_range_y[2], node_range_z[2];   
    KDTreeNode *left_son_ptr = nullptr;
    KDTreeNode *right_son_ptr = nullptr;
    KDTreeNode *father_ptr = nullptr;
    // For paper data record
    float alpha_del;
    float alpha_bal;
};

struct PointTypeCmp{
    PointType point;
    float dist = 0.0;
    PointTypeCmp (PointType p = ZeroP, float d = INFINITY){
        this->point = p;
        this->dist = d;
    };
    bool operator < (const PointTypeCmp &a)const{
        if (fabs(dist - a.dist) < 1e-10) return point.x < a.point.x;
          else return dist < a.dist;
    }    
};

struct BoxPointType{
    float vertex_min[3];
    float vertex_max[3];
};

enum OperationSet {ADD_POINT, DELETE_POINT, DELETE_BOX, ADD_BOX, DOWNSAMPLE_DELETE, PUSH_DOWN};

enum DeletePointStorageSet {NOT_RECORD, DELETE_POINTS_REC, MULTI_THREAD_REC};

struct OperationLoggerType{
    PointType point;
    BoxPointType boxpoint;
    bool tree_deleted, tree_downsample_deleted;
    OperationSet op;
};

class MANUAL_Q{
    private:
        int head = 0,tail = 0, counter = 0;
        OperationLoggerType q[Q_LEN];
        bool is_empty;
    public:
        void pop();
        OperationLoggerType front();
        OperationLoggerType back();
        void clear();
        void push(OperationLoggerType op);
        bool empty();
        int size();
};

class MANUAL_HEAP
{
    public:
        MANUAL_HEAP(int max_capacity = 100);
        ~MANUAL_HEAP();
        void pop();
        PointTypeCmp top();
        void push(PointTypeCmp point);
        int size();
        void clear();
    private:
        PointTypeCmp * heap;
        void MoveDown(int heap_index);
        void FloatUp(int heap_index);
        int heap_size = 0;
        int cap = 0;
};


class KDTree
{
private:
    // Multi-thread Tree Rebuild
    bool termination_flag = false;
    bool rebuild_flag = false;
    pthread_t rebuild_thread;
    pthread_mutex_t termination_flag_mutex_lock, rebuild_ptr_mutex_lock, working_flag_mutex, search_flag_mutex;
    pthread_mutex_t rebuild_logger_mutex_lock, points_deleted_rebuild_mutex_lock;
    // queue<OperationLoggerType> Rebuild_Logger;
    MANUAL_Q Rebuild_Logger;    
    PointVector Rebuild_PCL_Storage;
    KDTreeNode ** Rebuild_Ptr;
    int search_mutex_counter = 0;
    static void * multi_thread_ptr(void *arg);
    void multi_thread_rebuild();
    void start_thread();
    void stop_thread();
    void run_operation(KDTreeNode ** root, OperationLoggerType operation);
    // KD Tree Functions and augmented variables
    int Treesize_tmp = 0, Validnum_tmp = 0;
    float alpha_bal_tmp = 0.5, alpha_del_tmp = 0.0;
    float delete_criterion_param = 0.5f;
    float balance_criterion_param = 0.7f;
    float downsample_size = 0.2f;
    bool Delete_Storage_Disabled = false;
    KDTreeNode * STATIC_ROOT_NODE = nullptr;
    PointVector Points_deleted;
    PointVector Downsample_Storage;
    PointVector Multithread_Points_deleted;
    void InitTreeNode(KDTreeNode * root);
    void Test_Lock_States(KDTreeNode *root);
    void BuildTree(KDTreeNode ** root, int l, int r, PointVector & Storage);
    void Rebuild(KDTreeNode ** root);
    int Delete_by_range(KDTreeNode ** root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample);
    void Delete_by_point(KDTreeNode ** root, PointType point, bool allow_rebuild);
    void Add_by_point(KDTreeNode ** root, PointType point, bool allow_rebuild, int father_axis);
    void Add_by_range(KDTreeNode ** root, BoxPointType boxpoint, bool allow_rebuild);
    void Search(KDTreeNode * root, int k_nearest, PointType point, MANUAL_HEAP &q, double max_dist);//priority_queue<PointTypeCmp>
    void Search_by_range(KDTreeNode *root, BoxPointType boxpoint, PointVector &Storage);
    bool Criterion_Check(KDTreeNode * root);
    void Push_Down(KDTreeNode * root);
    void Update(KDTreeNode * root); 
    void delete_tree_nodes(KDTreeNode ** root);
    void downsample(KDTreeNode ** root);
    bool same_point(PointType a, PointType b);
    float calc_dist(PointType a, PointType b);
    float calc_box_dist(KDTreeNode * node, PointType point);    
    static bool point_cmp_x(PointType a, PointType b); 
    static bool point_cmp_y(PointType a, PointType b); 
    static bool point_cmp_z(PointType a, PointType b); 
    void print_treenode(KDTreeNode * root, int index, FILE *fp, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);

public:
    KDTree(float delete_param = 0.5, float balance_param = 0.6 , float box_length = 0.2);
    ~KDTree();
    void Set_delete_criterion_param(float delete_param);
    void Set_balance_criterion_param(float balance_param);
    void set_downsample_param(float box_length);
    void InitializeKDTree(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2); 
    int size();
    int validnum();
    void root_alpha(float &alpha_bal, float &alpha_del);
    void Build(PointVector point_cloud);
    void Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> & Point_Distance, double max_dist = INFINITY);
    int Add_Points(PointVector & PointToAdd, bool downsample_on);
    void Add_Point_Boxes(vector<BoxPointType> & BoxPoints);
    void Delete_Points(PointVector & PointToDel);
    int Delete_Point_Boxes(vector<BoxPointType> & BoxPoints);
    void flatten(KDTreeNode * root, PointVector &Storage, DeletePointStorageSet storage_type);
    void acquire_removed_points(PointVector & removed_points);
    void print_tree(int index, FILE *fp, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
    BoxPointType tree_range();
    PointVector PCL_Storage;     
    KDTreeNode * Root_Node = nullptr;
    int max_queue_size = 0;
};
