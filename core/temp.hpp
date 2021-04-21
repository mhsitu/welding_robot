#include <bits/stdc++.h>
using namespace std;


// const
const int INF = 0x3f3f3f3f;
#define sqr(x) ((x) * (x))
#define eps 1e-8
//variables
string file_name;
int type;           // type == 1 全矩阵, type == 2 二维欧拉距离
int N;              //城市数量
double **dis;       //城市间距离
double **pheromone; //信息素
double **herustic;  //启发式值
double **info;      // info = pheromone ^ delta * herustic ^ beta
double pheromone_0; //pheromone初始值，这里是1 / (avg * N)其中avg为图网中所有边边权的平均数。
int m;              //种群数量
int delta, beta;    //参数
double alpha;
int *r1, *s, *r;    //agent k的出发城市，下一个点，当前点。
int MAX, iteration; //最大迭代次数，迭代计数变量
set<int> empty, *J;
struct vertex
{
    double x, y; // 城市坐标
    int id;      // 城市编号

    int input(FILE *fp)
    {
        return fscanf(fp, "%d %lf %lf", &id, &x, &y);
    }

} * node;

typedef pair<int, int> pair_int;
struct Tour
{                          //路径
    vector<pair_int> path; //path[i]，存储一条边(r,s)
    double L;

    void clean()
    {
        L = INF;
        path.clear();
        path.shrink_to_fit();
    } //清空

    void calc()
    {
        L = 0;
        int sz = path.size();
        for (int i = 0; i < sz; i++)
        {
            L += dis[path[i].first][path[i].second];
        }
    } //计算长度

    void push_back(int x, int y)
    {
        path.push_back(make_pair(x, y));
    }

    int size()
    {
        return (int)path.size();
    }

    int r(int i)
    {
        return path[i].first;
    }

    int s(int i)
    {
        return path[i].second;
    }

    void print(FILE *fp)
    {
        int sz = path.size();
        for (int i = 0; i < sz; i++)
        {
            fprintf(fp, "%d->", path[i].first + 1);
        }
        fprintf(fp, "%d\n", path[sz - 1].second + 1);
    }

    bool operator<(const Tour &a) const
    {
        return L < a.L;
    } //重载

} * tour, best_so_far;

double EUC_2D(const vertex &a, const vertex &b)
{
    return sqrt(sqr(a.x - b.x) + sqr(a.y - b.y));
}

void io()
{ //输入
    printf("input file_name and data type\n");
    cin >> file_name >> type;
    FILE *fp = fopen(file_name.c_str(), "r");
    fscanf(fp, "%d", &N);
    node = new vertex[N + 5];
    dis = new double *[N + 5];
    double tmp = 0;
    int cnt = 0;
    if (type == 1)
    {
        for (int i = 0; i < N; i++)
        {
            dis[i] = new double[N];
            for (int j = 0; j < N; j++)
            {
                fscanf(fp, "%lf", &dis[i][j]);
                tmp += i != j ? dis[i][j] : 0; // i == j的时候 dis不存在，所以不考虑。
                cnt += i != j ? 1 : 0;         // i == j的时候 dis不存在，所以不考虑。
            }
        }
    }
    else
    {
        for (int i = 0; i < N; i++)
            node[i].input(fp);
        for (int i = 0; i < N; i++)
        {
            dis[i] = new double[N];
            for (int j = 0; j < N; j++)
            {
                dis[i][j] = EUC_2D(node[i], node[j]); // 计算距离
                tmp += i != j ? dis[i][j] : 0;        // i == j的时候 dis不存在，所以不考虑。
                cnt += i != j ? 1 : 0;                // i == j的时候 dis不存在，所以不考虑。
            }
        }
    }
    pheromone_0 = (double)cnt / (tmp * N); //pheromone初始值，这里是1 / (avg * N)其中avg为图网中所有边边权的平均数。
    fclose(fp);
    return;
}

void init()
{                //初始化
    alpha = 0.1; //evaporation parameter，挥发参数，每次信息素要挥发的量
    delta = 1;
    beta = 6; // delta 和 beta分别表示pheromones 和 herustic的比重
    m = N;
    pheromone = new double *[N + 5];
    herustic = new double *[N + 5];
    info = new double *[N + 5];
    r1 = new int[N + 5];
    r = new int[N + 5];
    s = new int[N + 5];
    J = new set<int>[N + 5];
    empty.clear();
    for (int i = 0; i < N; i++)
    {
        pheromone[i] = new double[N + 5];
        herustic[i] = new double[N + 5];
        info[i] = new double[N + 5];
        empty.insert(i);
        for (int j = 0; j < N; j++)
        {
            pheromone[i][j] = pheromone_0;
            herustic[i][j] = 1 / (dis[i][j] + eps); //加一个小数eps，防止被零除
        }
    }
    best_so_far.clean();
    iteration = 0;
    MAX = N * N;
}

double power(double x, int y)
{ //快速幂，计算x ^ y，时间复杂度O(logn),感兴趣可以百度
    double ans = 1;
    while (y)
    {
        if (y & 1)
            ans *= x;
        x *= x;
        y >>= 1;
    }
    return ans;
}

void reset()
{
    tour = new Tour[m + 5];
    for (int i = 0; i < N; i++)
    {
        tour[i].clean();
        r1[i] = i; //初始化出发城市，
        J[i] = empty;
        J[i].erase(r1[i]); //初始化agent i需要访问的城市
        r[i] = r1[i];      //当前在出发点
    }
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
        {
            info[i][j] = power(pheromone[i][j], delta) * power(herustic[i][j], beta);
        } //选择公式
}

int select_next(int k)
{
    if (J[k].empty())
        return r1[k];                                 //如果J是空的，那么返回出发点城市
    double rnd = (double)(rand()) / (double)RAND_MAX; //产生0..1的随机数
    set<int>::iterator it = J[k].begin();
    double sum_prob = 0, sum = 0;
    for (; it != J[k].end(); it++)
    {
        sum += info[r[k]][*it];
    } //计算概率分布
    rnd *= sum;
    it = J[k].begin();
    for (; it != J[k].end(); it++)
    {
        sum_prob += info[r[k]][*it];
        if (sum_prob >= rnd)
        {
            return *it;
        }
    } //依照概率选取下一步城市
}

void construct_solution()
{
    for (int i = 0; i < N; i++)
    {
        for (int k = 0; k < m; k++)
        {
            int next = select_next(k); //选择下一步的最优决策
            J[k].erase(next);
            s[k] = next;
            tour[k].push_back(r[k], s[k]);
            r[k] = s[k];
        }
    }
}

void update_pheromone()
{
    Tour now_best;
    now_best.clean(); //初始化
    for (int i = 0; i < m; i++)
    {
        tour[i].calc();
        if (tour[i] < now_best)
            now_best = tour[i]; //寻找当前迭代最优解
    }
    if (now_best < best_so_far)
    {
        best_so_far = now_best; //更新最优解
    }
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            pheromone[i][j] *= (1 - alpha); //信息素挥发

    int sz = now_best.size();
    for (int i = 0; i < sz; i++)
    {
        pheromone[now_best.r(i)][now_best.s(i)] += 1. / (double)now_best.L;
        pheromone[now_best.s(i)][now_best.r(i)] = pheromone[now_best.r(i)][now_best.s(i)]; //对称
    }                                                                                      //更新信息素含量
}

int main()
{
    srand((unsigned)time(0)); //初始化随机种子
    io();
    init();
    double last = INF;
    int bad_times = 0;
    for (; iteration < MAX; iteration++)
    {
        if (bad_times > N)
            break;            //进入局部最优
        reset();              //初始化agent信息
        construct_solution(); //对于所有的agent构造一个完整的tour
        update_pheromone();   //更新信息素
        printf("iteration %d:best_so_far = %.2lf\n", iteration, best_so_far.L);
        if (last > best_so_far.L)
            last = best_so_far.L, bad_times = 0;
        else
            bad_times++; //记录当前未更新代数，若迭代多次未更新，认为进入局部最优
    }
    printf("best_so_far = %.2lf\n", best_so_far.L); //输出目标值
    best_so_far.print(stdout);                      //输出路径
}
