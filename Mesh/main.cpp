#include <iostream>
#include <vector>
#include <queue>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <list>
#include <memory>
#include <fstream>
#include <sstream>
#include <limits>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/LU>
/*
 * Command line DEFINES
 */
DEFINE_string(src, "", "Source OBJ File");
DEFINE_string(dst, "out.obj", "Target OBJ File");
DEFINE_double(ratio, 1., "Target simplification ratio");

/*
 * Simplification stuffs
 */
struct Face;

struct Point {
    Eigen::Vector3d p, n;
    Eigen::MatrixXd Q;
    std::unordered_set<std::shared_ptr<Point>> adj_pnts;
    std::unordered_set<std::shared_ptr<Face>> tri_faces;
    size_t idx; // for output use
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointPair {
    std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>> pair;
    float cost;
    std::shared_ptr<Point> desired;
};

struct SymmetricPointPairHash {
public:
    size_t operator()(const std::shared_ptr<PointPair> &p) const {
        return std::hash<std::shared_ptr<Point>>()(p->pair.first) ^ std::hash<std::shared_ptr<Point>>()(
                p->pair.second);
    }
};

struct PointStdPairHash {
public:
    size_t operator()(const std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>> &p) const {
        return std::hash<std::shared_ptr<Point>>()(p.first) ^ std::hash<std::shared_ptr<Point>>()(
                p.second);
    }
};

struct PointPairEqual {
public:
    bool operator()(const std::shared_ptr<PointPair> &p1, const std::shared_ptr<PointPair> &p2) const {
        return (p1->pair.first == p2->pair.first && p1->pair.second == p2->pair.second) ||
               (p1->pair.first == p2->pair.second && p1->pair.second == p2->pair.first);
    }
};

struct PointStdPairEqual {
    typedef std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>> PointStdPair;

    bool operator()(const PointStdPair &p1, const PointStdPair &p2) const {
        return (p1.first == p2.first && p1.second == p2.second) ||
               (p1.first == p2.second && p1.second == p2.first);
    }
};

struct Face {
    std::weak_ptr<Point> v1, v2, v3;

    bool contain(const std::shared_ptr<Point> &p) const {
        return p == v1.lock() || p == v2.lock() || p == v3.lock();
    }
};

class PointPairHeap {
private:
    std::unordered_map<std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>>, size_t, PointStdPairHash, PointStdPairEqual> _map;
    std::vector<std::shared_ptr<PointPair>> _heap;

    inline bool compare(size_t l, size_t r) {
        return _heap[l]->cost < _heap[r]->cost;
    }

    inline size_t LEFT(size_t p) {
        return (p << 1) + 1;
    }

    inline size_t RIGHT(size_t p) {
        return (p << 1) + 2;
    }

    inline size_t PARENT(size_t p) {
        return (p - 1) >> 1;
    }

    void percolateDown(size_t pos) {
        while (pos < _heap.size()) {
            int _min = pos;
            if (LEFT(pos) < _heap.size() && compare(LEFT(pos), _min))
                _min = LEFT(pos);
            if (RIGHT(pos) < _heap.size() && compare(RIGHT(pos), _min))
                _min = RIGHT(pos);
            if (_min == pos)
                break;
            else {
                std::swap(_heap[_min], _heap[pos]);
                updateIndex(_min);
                updateIndex(pos);
                pos = _min;
            }
        }
    }

    void percolateUp(size_t pos) {
        while (pos) {
            if (compare(pos, PARENT(pos))) {
                std::swap(_heap[pos], _heap[PARENT(pos)]);
                updateIndex(pos);
                updateIndex(PARENT(pos));
                pos = PARENT(pos);
            } else break;
        }
    }

    void updateIndex(size_t idx) {
        auto &v1 = _heap[idx]->pair.first, &v2 = _heap[idx]->pair.second;
        _map[{v1, v2}] = idx;
    }

    void removeIndex(size_t idx) {
        auto &v1 = _heap[idx]->pair.first, &v2 = _heap[idx]->pair.second;
        _map.erase({v1, v2});
    }

public:
    template<typename T>
    void Initialize(T iter_begin, T iter_end) {
        _heap = std::vector<std::shared_ptr<PointPair>>(iter_begin, iter_end);
        std::make_heap(_heap.begin(), _heap.end(),
                       [](const std::shared_ptr<PointPair> &p1, const std::shared_ptr<PointPair> &p2) {
                           return p1->cost > p2->cost;
                       });
        for (size_t i = 0; i < _heap.size(); ++i) {
            updateIndex(i);
        }
    }

    std::shared_ptr<PointPair> getTop() {
        if (_heap.size() == 0)
            return nullptr;
        else if (_heap.size() == 1) {
            auto v = _heap[0];
            _heap.clear();
            _map.clear();
            return v;
        } else {
            std::swap(_heap[0], _heap[_heap.size() - 1]);
            updateIndex(0);
            removeIndex(_heap.size() - 1);
            auto v = _heap[_heap.size() - 1];
            _heap.pop_back();
            // percolate down
            percolateDown(0);
            return v;
        }
    }

    void addPair(std::shared_ptr<PointPair> &pair) {
        _heap.emplace_back(pair);
        updateIndex(_heap.size() - 1);
        percolateUp(_heap.size() - 1);
    }

    void removePair(std::shared_ptr<Point> p1, std::shared_ptr<Point> p2) {
        auto it = _map.find({p1, p2});
        if (it != _map.end()) {
            size_t pos = it->second;
            _map.erase(it);
            if (pos == _heap.size() - 1)
                _heap.pop_back();
            else {
                std::swap(_heap[pos], _heap[_heap.size() - 1]);
                updateIndex(pos);
                _heap.pop_back();
                percolateDown(pos);
            }
        }
    }
};

class Model {
    std::unordered_set<std::shared_ptr<Point>> vertices;
    PointPairHeap pp_heap;
    size_t num_faces = 0;
    bool has_norm = false;
    Eigen::Vector3d scale, pmin, pmax;
    /** Statistics **/
    size_t flip_num = 0, degenerate_num = 0;

    Eigen::VectorXd getVertAttr(std::shared_ptr<Point> p) {
        std::vector<double> vec;
        for (int i = 0; i < 3; ++i)
            vec.emplace_back(p->p[i]);
        if (has_norm) {
            for (int i = 0; i < 3; ++i)
                vec.emplace_back(p->n[i]);
        }
        Eigen::VectorXd res = Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
        return res;
    }

    float getCost(std::shared_ptr<PointPair> pair) {
        auto &v1 = pair->pair.first;
        auto &v2 = pair->pair.second;
        Eigen::MatrixXd Q = v1->Q + v2->Q;
        Eigen::MatrixXd dQ = Q;
        dQ.row(dQ.rows() - 1).setZero();
        dQ(dQ.rows() - 1, dQ.cols() - 1) = 1.;
        Eigen::FullPivLU<Eigen::MatrixXd> lu(dQ);
        Eigen::VectorXd v;
        if (lu.isInvertible()) {
            Eigen::VectorXd t(Q.cols());
            t.fill(0);
            t[t.size() - 1] = 1.;
            v = lu.inverse() * t;
        } else {
            auto v1_attr = getVertAttr(v1), v2_attr = getVertAttr(v2);
            Eigen::VectorXd v1v2 = v1_attr - v2_attr;
            Eigen::MatrixXd A = Q.topLeftCorner(Q.rows() - 1, Q.cols() - 1);
            Eigen::VectorXd b = Q.topRightCorner(Q.rows() - 1, 1);
            if (abs(v1v2.dot(A * v1v2)) > 1e-7) {
                double t = v2_attr.dot(A * v1v2) - b.dot(v1v2);
                if (t > 1) t = 1;
                if (t < 0) t = 0;
                Eigen::VectorXd vv = t * v1_attr + (1 - t) * v2_attr;
                v = Eigen::VectorXd(vv.size() + 1);
                v.fill(1);
                v.topRows(vv.size()) = vv;
            } else {
                // choose from end points or mid point
                // use homogeneous coord here
                Eigen::VectorXd v1_(v1_attr.size() + 1), v2_(v2_attr.size() + 1);
                v1_.topRows(v1_attr.size()) = v1_attr, v2_.topRows(v2_attr.size()) = v2_attr;
                v1_(v1_attr.size()) = 1, v2_(v2_attr.size()) = 1;
                Eigen::VectorXd v_mid_ = .5 * v1_ + .5 * v2_;
                double cost_1 = v1_.transpose() * Q * v1_, cost_2 = v2_.transpose() * Q * v2_,
                        cost_mid = v_mid_.transpose() * Q * v_mid_;
                if (cost_1 < cost_2) {
                    if (cost_1 < cost_mid)
                        v = v1_;
                    else
                        v = v_mid_;
                } else {
                    if (cost_2 < cost_mid)
                        v = v2_;
                    else
                        v = v_mid_;
                }
            }
        }

        if (!pair->desired) {
            pair->desired = std::shared_ptr<Point>(new Point());
            pair->desired->Q = Q;
            pair->desired->p = v.topRows(3);
            if (has_norm) {
                pair->desired->n = v.segment(3, 3);
                pair->desired->n.normalize();
            }
        }

        return v.transpose() * Q * v;
    }

    void mergePointPair(std::shared_ptr<PointPair> pair) {
        using namespace std;
        auto v1 = pair->pair.first, v2 = pair->pair.second, desired = pair->desired;

        // maintain faces
        unordered_set<shared_ptr<Face>> faces;
        for (auto &f: v1->tri_faces)
            faces.insert(f);
        for (auto &f: v2->tri_faces)
            faces.insert(f);

        // Flip check !!!
        bool flip = false;
        for (auto &f: faces) {
            Face temp = *f;
            if (temp.contain(v1)) {
                if (temp.v1.lock() == v1)
                    temp.v1 = desired;
                else if (temp.v2.lock() == v1)
                    temp.v2 = desired;
                else if (temp.v3.lock() == v1)
                    temp.v3 = desired;
            } else if (temp.contain(v2)) {
                if (temp.v1.lock() == v2)
                    temp.v1 = desired;
                else if (temp.v2.lock() == v2)
                    temp.v2 = desired;
                else if (temp.v3.lock() == v2)
                    temp.v3 = desired;
            }
            Eigen::Vector3d n1 = (f->v1.lock()->p - f->v2.lock()->p).cross(f->v1.lock()->p - f->v3.lock()->p), n2 = (
                    temp.v1.lock()->p - temp.v2.lock()->p).cross(
                    temp.v1.lock()->p - temp.v3.lock()->p);
            n1.normalize();
            n2.normalize();
            if (n1.dot(n2) < 0.2) {
                flip = true;
                break;
            }
        }

        if (flip) {
            // Don't contract this pair if a face flip occurs
            ++flip_num;
            return;
        }

        for (auto &f: faces) {
            if (f->contain(v1) && f->contain(v2)) {
                // this face should be removed
                if (f->v1.lock() != v1 && f->v1.lock() != v2) {
                    f->v1.lock()->tri_faces.erase(f);
                    if (f->v1.lock()->tri_faces.size() == 0 ^ f->v1.lock()->adj_pnts.size() == 0) {
                        ++degenerate_num;
                    }
                } else if (f->v2.lock() != v1 && f->v2.lock() != v2) {
                    f->v2.lock()->tri_faces.erase(f);
                    if (f->v2.lock()->tri_faces.size() == 0 ^ f->v2.lock()->adj_pnts.size() == 0) {
                        ++degenerate_num;
                    }
                } else if (f->v3.lock() != v1 && f->v3.lock() != v2) {
                    f->v3.lock()->tri_faces.erase(f);
                    if (f->v3.lock()->tri_faces.size() == 0 ^ f->v3.lock()->adj_pnts.size() == 0) {
                        ++degenerate_num;
                    }
                }
            } else {
                if (f->contain(v1)) {
                    if (f->v1.lock() == v1)
                        f->v1 = desired;
                    else if (f->v2.lock() == v1)
                        f->v2 = desired;
                    else if (f->v3.lock() == v1)
                        f->v3 = desired;
                } else if (f->contain(v2)) {
                    if (f->v1.lock() == v2)
                        f->v1 = desired;
                    else if (f->v2.lock() == v2)
                        f->v2 = desired;
                    else if (f->v3.lock() == v2)
                        f->v3 = desired;
                }
                desired->tri_faces.insert(f);
            }
        }

        unordered_set<shared_ptr<Point>> adjacent;
        // merge the adjacent pnts of v1, v2; delete v1, v2 from their adjacencies
        for (auto &v: v1->adj_pnts) {
            if (v != v2) {
                adjacent.insert(v);
            }
            v->adj_pnts.erase(v1);
            pp_heap.removePair(v, v1);
        }
        for (auto &v: v2->adj_pnts) {
            if (v != v1) {
                adjacent.insert(v);
            }
            v->adj_pnts.erase(v2);
            pp_heap.removePair(v, v2);
        }
        desired->adj_pnts = adjacent;
        for (auto &v: adjacent) {
            v->adj_pnts.insert(desired);

            shared_ptr<PointPair> new_pair(new PointPair);
            new_pair->pair = {desired, v};
            new_pair->cost = getCost(new_pair);
            pp_heap.addPair(new_pair);
        }

        vertices.erase(v1);
        vertices.erase(v2);
        vertices.insert(desired);
    }

public:
    Model(std::string
          file) {
        using namespace std;
        std::vector<std::shared_ptr<Point>> vertices_buf;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> norms_buf;
        fstream fin;
        fin.open(file, fstream::in);
        if (fin.fail()) {
            LOG(FATAL) << "Fail to load obj file";
        }
        constexpr size_t BUF_LIMIT = 1024;
        char buf[BUF_LIMIT];
        constexpr double double_max = numeric_limits<double>::max(), double_min = numeric_limits<double>::min();
        Eigen::Vector3d p_max(double_min, double_min, double_min), p_min(double_max, double_max, double_max);
        while (fin.getline(buf, BUF_LIMIT)) {
            stringstream in(buf);
            string token;
            in >> token;
            if (token == "v") {
                Eigen::Vector3d vert;
                in >> vert[0] >> vert[1] >> vert[2];
                for (size_t i = 0; i < 3; ++i) {
                    p_max[i] = max(p_max[i], vert[i]);
                    p_min[i] = min(p_min[i], vert[i]);
                }
                auto p = std::shared_ptr<Point>(new Point());
                p->p = vert;
                vertices_buf.emplace_back(p);
            } else if (token == "vn") {
                Eigen::Vector3d n;
                in >> n[0] >> n[1] >> n[2];
                n.normalize();
                norms_buf.emplace_back(n);
            } else if (token == "f") {
                ++num_faces;
                // triangular face
                string seg;
                int pp = 0;
                shared_ptr<Face> face(new Face());
                while (in >> seg) {
                    ++pp;
                    auto pos1 = seg.find('/');
                    auto pos2 = seg.find('/', pos1 + 1);
                    string v = seg.substr(0, pos1), vt = (pos1 >= seg.length()) ? ""
                                                                                : seg.substr(
                                    pos1 + 1, pos2 - pos1 - 1), vn = (pos2 >= seg.length()) ? "" : seg.substr(
                            pos2 + 1);
                    if (!v.length()) {
                        LOG(FATAL) << "Invalid OBJ File. Faces should contain vertex info at least!";
                    }
                    auto vv = stoi(v) - 1;
                    if (vn.length()) {
                        vertices_buf[vv]->n = norms_buf[stoi(vn) - 1];
                    }
                    switch (pp) {
                        case 1:
                            face->v1 = vertices_buf[vv];
                            break;
                        case 2:
                            face->v2 = vertices_buf[vv];
                            break;
                        case 3:
                            face->v3 = vertices_buf[vv];
                            break;
                    }
                    vertices_buf[vv]->tri_faces.insert(face);
                }
                // handle adjacency
                if (face->v1.lock()->adj_pnts.find(face->v2.lock()) == face->v1.lock()->adj_pnts.end())
                    face->v1.lock()->adj_pnts.insert(face->v2.lock());
                if (face->v1.lock()->adj_pnts.find(face->v3.lock()) == face->v1.lock()->adj_pnts.end())
                    face->v1.lock()->adj_pnts.insert(face->v3.lock());
                if (face->v2.lock()->adj_pnts.find(face->v1.lock()) == face->v2.lock()->adj_pnts.end())
                    face->v2.lock()->adj_pnts.insert(face->v1.lock());
                if (face->v2.lock()->adj_pnts.find(face->v3.lock()) == face->v2.lock()->adj_pnts.end())
                    face->v2.lock()->adj_pnts.insert(face->v3.lock());
                if (face->v3.lock()->adj_pnts.find(face->v1.lock()) == face->v3.lock()->adj_pnts.end())
                    face->v3.lock()->adj_pnts.insert(face->v1.lock());
                if (face->v3.lock()->adj_pnts.find(face->v2.lock()) == face->v3.lock()->adj_pnts.end())
                    face->v3.lock()->adj_pnts.insert(face->v2.lock());
            }
            // we don't handle vt texture here
        }
        has_norm = norms_buf.size() != 0;
        fin.close();
        LOG(INFO) << "Load OBJ Successfully with " << vertices_buf.size() << " vertices_buf, " << num_faces
                  << " faces";

        // normalize points
        scale = p_max - p_min;
        for (auto &v:vertices_buf) {
            v->p = v->p - p_min;
            v->p = v->p.cwiseQuotient(scale);
        }

        pmin = p_min, pmax = p_max;

        // initialize Q Matrix
        for (auto &v : vertices_buf) {
            Eigen::MatrixXd Q;
            for (auto &f: v->tri_faces) {
                Eigen::VectorXd v1 = getVertAttr(f->v1.lock());
                Eigen::VectorXd v2 = getVertAttr(f->v2.lock());
                Eigen::VectorXd v3 = getVertAttr(f->v3.lock());
                Eigen::VectorXd e1 = v2 - v1;
                e1.normalize();
                Eigen::VectorXd e2 = v3 - v1 - e1.dot(v3 - v1) * e1;
                e2.normalize();

                size_t dim = static_cast<size_t>(v1.size());
                Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim, dim) - e1 * e1.transpose() - e2 * e2.transpose();
                double v1e1 = v1.dot(e1), v1e2 = v1.dot(e2);
                Eigen::VectorXd b = v1e1 * e1 + v1e2 * e2 - v1;
                double c = v1.dot(v1) - v1e1 * v1e1 - v1e2 * v1e2;
                Eigen::MatrixXd Ab(A.rows(), A.cols() + b.cols());
                Ab << A, b;
                Eigen::MatrixXd bc(b.size() + 1, 1);
                bc << b, c;
                Eigen::MatrixXd K(Ab.rows() + 1, Ab.cols());
                K << Ab, bc.transpose();
                if (!Q.size())
                    Q = K;
                else Q += K;
            }

            v->Q = Q;
        }

        LOG(INFO) << "Q Matrix initialized, now construct point pairs..";

        // Construct pairs
        size_t tot = 0;
        std::unordered_set<std::shared_ptr<PointPair>, SymmetricPointPairHash, PointPairEqual> pairs;
        for (auto &v : vertices_buf) {
            tot += v->adj_pnts.size();
            for (auto &adj : v->adj_pnts) {
                auto pair = shared_ptr<PointPair>(new PointPair());
                pair->pair = {v, adj};
                if (pairs.find(pair) == pairs.end()) {
                    pair->cost = getCost(pair); // evalCost
                    pairs.insert(pair);
                }
            }
        }

        pp_heap.Initialize(pairs.begin(), pairs.end());

        CHECK((pairs.size() << 1) == tot) << "Internal error!";

        vertices = unordered_set<shared_ptr<Point>>(vertices_buf.begin(), vertices_buf.end());

        LOG(INFO) << "Initialization finished";
    }

    void simplify(std::string out_file, float ratio) {
        using namespace std::chrono;
        flip_num = degenerate_num = 0;
        CHECK(ratio > 0 && ratio <= 1) << "Invalid simplification ratio detected. Should in range (0, 1]";
        int target_num_vert = static_cast<int>(ratio * vertices.size());
        auto start = high_resolution_clock::now();
        while (vertices.size() > target_num_vert) {
            auto pp = pp_heap.getTop();
            mergePointPair(pp);
        }
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        LOG(INFO) << "Simplification finished with " << vertices.size() << " vertices left. Time consumption: "
                  << duration.count() << " ms";
        LOG(INFO) << "During this process, " << flip_num << " face flips were prevented, " << degenerate_num
                  << " points were degenerated.";
        LOG(INFO) << "Saving to file " << out_file;
        using namespace std;
        fstream fout;
        fout.open(out_file, fstream::out | fstream::trunc);
        CHECK(!fout.fail()) << "Fail to open file to write.";
        std::vector<std::shared_ptr<Point>> result_vert(vertices.begin(), vertices.end());
        unordered_set<shared_ptr<Face>> faces;
        fout << "# Simplified model with " << vertices.size() << " vertices" << endl;
        for (size_t i = 0; i < result_vert.size(); ++i) {
            auto &v = result_vert[i];
            v->idx = i;
            for (auto &f : v->tri_faces) {
                if (faces.find(f) == faces.end())
                    faces.insert(f);
            }
            Eigen::Vector3d vv = v->p.cwiseProduct(scale) + pmin;
            fout << "v " << vv.x() << " " << vv.y() << " " << vv.z() << endl;
        }
        if (has_norm) {
            for (auto &v: result_vert) {
                fout << "vn " << v->n.x() << " " << v->n.y() << " " << v->n.z() << endl;
            }
        }
        fout << "# Faces: " << faces.size() << endl;
        LOG(INFO) << "Total " << faces.size() << " faces";
        for (auto &f: faces) {
            fout << "f ";
            auto v1 = f->v1.lock();
            auto v2 = f->v2.lock();
            auto v3 = f->v3.lock();
            auto output = [&](shared_ptr<Point> &p) {
                if (has_norm)
                    fout << p->idx + 1 << "//" << p->idx + 1;
                else
                    fout << p->idx + 1 << "//";
            };
            output(v1);
            fout << " ";
            output(v2);
            fout << " ";
            output(v3);
            fout << endl;
        }
        fout.close();
    }
};

int main(int argc, char **argv) {
    gflags::SetUsageMessage("Mesh simplification by zx1239856.");
    gflags::SetVersionString("Version 0.1.0 alpha");
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    google::LogToStderr();
    if (FLAGS_src.empty() || FLAGS_dst.empty() || FLAGS_ratio > 1 || FLAGS_ratio < 0) {
        LOG(ERROR) << "Invalid parameter, please refer to usage using: " << argv[0] << " --help";
    }
    Model model(FLAGS_src);
    model.simplify(FLAGS_dst, FLAGS_ratio);
    return 0;
}

