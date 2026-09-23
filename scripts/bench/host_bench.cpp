// Portable host benchmark for the SwarmDeck clean-up plan.
// Reproduces MGG's NativeMolaGrid::status hot spot (two binary searches over
// sorted 24-byte cells) and a hashed alternative, plus generic CPU/memory.
// Build: g++ -O2 -std=c++17 -pthread host_bench.cpp -o host_bench
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <thread>
#include <unordered_set>
#include <vector>

using clk = std::chrono::steady_clock;
static double secs(clk::time_point a) {
  return std::chrono::duration<double>(clk::now() - a).count();
}

struct Cell {
  std::int64_t x = 0, y = 0, z = 0;
  friend bool operator<(const Cell& a, const Cell& b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  }
  friend bool operator==(const Cell& a, const Cell& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
  }
};
struct CellHash {
  std::size_t operator()(const Cell& c) const {
    std::uint64_t h = std::uint64_t(c.x) * 0x9E3779B97F4A7C15ull;
    h ^= std::uint64_t(c.y) * 0xC2B2AE3D27D4EB4Full + (h << 6) + (h >> 2);
    h ^= std::uint64_t(c.z) * 0x165667B19E3779F9ull + (h << 6) + (h >> 2);
    return h;
  }
};

// A tunnel-like world: occupied walls/floor, free interior, over an extent.
static void makeGrid(std::size_t target, std::vector<Cell>& occ,
                     std::vector<Cell>& fre, std::mt19937_64& rng) {
  std::uniform_int_distribution<int> d(-2000, 2000), dz(-10, 30);
  occ.reserve(target / 3);
  fre.reserve(target);
  std::unordered_set<Cell, CellHash> seen;
  seen.reserve(target * 2);
  while (occ.size() + fre.size() < target) {
    Cell c{d(rng), d(rng), dz(rng)};
    // grow short corridors so neighbouring cells exist (locality like a map)
    for (int i = 0; i < 64 && occ.size() + fre.size() < target; ++i) {
      Cell k{c.x + i, c.y, c.z};
      for (int w = 0; w < 4; ++w) {
        Cell q{k.x, k.y + w, k.z};
        if (!seen.insert(q).second) continue;
        (w == 0 || w == 3 ? occ : fre).push_back(q);
      }
    }
  }
  std::sort(occ.begin(), occ.end());
  std::sort(fre.begin(), fre.end());
}

// Ray-like coherent queries: walk 40 cells from random starts near data.
static std::vector<Cell> makeQueries(const std::vector<Cell>& fre,
                                     std::size_t n, std::mt19937_64& rng) {
  std::vector<Cell> q;
  q.reserve(n);
  std::uniform_int_distribution<std::size_t> pick(0, fre.size() - 1);
  std::uniform_int_distribution<int> dir(-1, 1);
  while (q.size() < n) {
    Cell c = fre[pick(rng)];
    int dx = dir(rng), dy = dir(rng), dz = dir(rng);
    for (int i = 0; i < 40 && q.size() < n; ++i)
      q.push_back({c.x + dx * i, c.y + dy * i, c.z + dz * i});
  }
  return q;
}

static int statusBS(const std::vector<Cell>& o, const std::vector<Cell>& f,
                    const Cell& k) {
  if (std::binary_search(o.begin(), o.end(), k)) return 2;
  if (std::binary_search(f.begin(), f.end(), k)) return 1;
  return 0;
}

template <class F>
static double runThreads(int threads, F body) {
  auto t = clk::now();
  std::vector<std::thread> ts;
  for (int i = 0; i < threads; ++i) ts.emplace_back(body, i);
  for (auto& x : ts) x.join();
  return secs(t);
}

int main(int argc, char** argv) {
  const int hw = int(std::thread::hardware_concurrency());
  std::mt19937_64 rng(42);
  std::printf("threads_available %d\n", hw);

  // 1-2. MGG voxel status: binary search vs hash, single thread.
  for (std::size_t cells : {250000ul, 1000000ul, 4000000ul}) {
    std::vector<Cell> occ, fre;
    makeGrid(cells, occ, fre, rng);
    auto qs = makeQueries(fre, 4000000, rng);
    std::unordered_set<Cell, CellHash> ho(occ.begin(), occ.end()),
        hf(fre.begin(), fre.end());
    volatile long sink = 0;
    long s = 0;
    for (int rep = 0; rep < 1; ++rep) for (auto& k : qs) s += statusBS(occ, fre, k);  // warm
    auto t = clk::now();
    s = 0;
    for (int rep = 0; rep < 3; ++rep)
      for (auto& k : qs) s += statusBS(occ, fre, k);
    double bs = secs(t) / (3.0 * qs.size()) * 1e9;
    sink = sink + s;
    t = clk::now();
    s = 0;
    for (int rep = 0; rep < 3; ++rep)
      for (auto& k : qs) s += ho.count(k) ? 2 : (hf.count(k) ? 1 : 0);
    double hs = secs(t) / (3.0 * qs.size()) * 1e9;
    sink = sink + s;
    // all threads, binary search (throughput when other processes compete)
    std::atomic<long> tot{0};
    double wall = runThreads(hw, [&](int id) {
      long l = 0;
      for (int rep = 0; rep < 3; ++rep)
        for (std::size_t i = id; i < qs.size(); i += hw) l += statusBS(occ, fre, qs[i]);
      tot += l;
    });
    double mt = wall / (3.0 * qs.size()) * 1e9;
    std::printf("voxel_status cells=%zu bsearch_ns=%.1f hash_ns=%.1f bsearch_allthreads_ns_per_query=%.2f\n",
                occ.size() + fre.size(), bs, hs, mt);
  }

  // 3. Generic CPU: sort 10M doubles, single thread.
  {
    std::vector<double> v(10000000);
    std::uniform_real_distribution<double> u;
    for (auto& x : v) x = u(rng);
    auto t = clk::now();
    std::sort(v.begin(), v.end());
    std::printf("sort_10M_doubles_ms %.0f\n", secs(t) * 1e3);
  }
  // 4. Generic FP: naive-blocked 768^3 float matmul, single thread.
  {
    const int n = 768;
    std::vector<float> a(n * n), b(n * n), c(n * n, 0);
    std::uniform_real_distribution<float> u;
    for (auto& x : a) x = u(rng);
    for (auto& x : b) x = u(rng);
    auto t = clk::now();
    for (int i = 0; i < n; ++i)
      for (int k = 0; k < n; ++k) {
        float aik = a[i * n + k];
        for (int j = 0; j < n; ++j) c[i * n + j] += aik * b[k * n + j];
      }
    double s = secs(t);
    std::printf("matmul768_single_gflops %.2f (c0=%g)\n", 2.0 * n * n * n / s / 1e9, c[0]);
  }
  // 5. Memory bandwidth: 512 MiB copy, 1 thread and all threads.
  {
    const std::size_t bytes = 512ull << 20;
    std::vector<char> src(bytes, 1), dst(bytes, 0);
    auto t = clk::now();
    for (int r = 0; r < 4; ++r) std::memcpy(dst.data(), src.data(), bytes);
    double one = 4.0 * 2 * bytes / secs(t) / 1e9;
    double wall = runThreads(hw, [&](int id) {
      std::size_t chunk = bytes / hw, off = chunk * id;
      for (int r = 0; r < 4; ++r) std::memcpy(dst.data() + off, src.data() + off, chunk);
    });
    double all = 4.0 * 2 * bytes / wall / 1e9;
    std::printf("memcpy_GBps single %.1f all_threads %.1f\n", one, all);
  }
  return 0;
}
