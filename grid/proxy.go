package grid

import (
	"sync"
	"sync/atomic"

	"github.com/actfuns/pathfinding/finder"
)

// Proxy is the read-only Grid view a scene holds for the lifetime of its map.
//
// It exposes the read-only Grid operations (ReadOnlyGrid) and splits the two
// access patterns:
//
//   - FindPath and FindPathWorld lease an instance from the Pool, run the search
//     on it and hand it straight back, so that searches from different scenes —
//     which share the Pool — never share finder state. The result is copied into
//     the Proxy's own buffer before the instance goes back, because another
//     Proxy may lease that very instance and overwrite its buffer immediately.
//   - Every other method is forwarded to the Pool's resident Base instance, so
//     reads never block on a lease.
//
// # Concurrency
//
// ONE PROXY, ONE GOROUTINE. A Proxy is not safe for concurrent use.
//
// The Proxy caches the Base it reads from and owns the buffer that every result
// is copied into, so two goroutines driving the same Proxy corrupt both: a
// search overwrites the other's path while it is still being read. Nothing in
// here detects that — it fails silently, returning a mangled path.
//
// The fix is not a lock but another Proxy. Parallelism comes from several
// Proxies sharing one Pool, which is what the Pool is for:
//
//	m := grid.NewPoolManager()
//	// once per scene, then used only by that scene's goroutine
//	pr, err := m.Acquire("town", factory)
//
// Concurrent safety, precisely:
//
//   - Different Proxies, any number of goroutines — safe. This is the supported
//     concurrent shape.
//   - One Proxy, one goroutine, however many concurrent Pool users — safe.
//   - One Proxy, several goroutines — UNSAFE. Silent corruption, no panic.
//
// # Path lifetime
//
// Like the grids behind it, the returned path is backed by the Proxy's buffer and
// stays valid only until the next FindPath/FindPathWorld call on the same Proxy.
// Copy or convert it before the next search.
//
// # Release and Reset
//
// Release must be called exactly once when the scene is done; extra calls are
// safe no-ops. A released Proxy is dead, not merely quiet: every Proxy method
// panics rather than answering from a map that is gone. Release also drops the
// cached Base and result buffers so they can be reclaimed.
//
// Proxy is read-only. To change map data, build a new Factory and call
// PoolManager.Reset; mutations are deliberately not exposed because leased
// instances are independent copies and would not see them.
//
// PoolManager.Reset does NOT do that. It swaps the map data underneath and
// leaves this Proxy live and in use — existing Proxies keep working and serve
// the new map from their next call onward, with no re-Acquire and no panic.
type Proxy interface {
	ReadOnlyGrid
	Release()
}

// proxy implements Proxy: read-only calls are forwarded to the resident Base
// instance, pathfinding goes through a lease and lands in a reused buffer.
// There are no mutators.
//
// Everything below the Pool handle is unsynchronised state (base, baseEpoch,
// pathBuf, worldBuf) and is written without atomics on the assumption of ONE
// GOROUTINE PER PROXY. See the Proxy interface doc: parallel scenes share the
// Pool, not the Proxy. Breaking that assumption corrupts results silently, so
// the fields are deliberately plain — an atomic here would cost more than the
// contract it pretended to enforce.
type proxy struct {
	pool    *Pool
	manager *PoolManager
	key     string

	pathBuf  [][2]int
	worldBuf [][2]float32

	base        ReadOnlyGrid // cached pool.Base(), filled on first read
	baseEpoch   uint64       // Pool.Epoch() that p.base was read from
	released    atomic.Bool
	releaseOnce sync.Once
}

// checkLive panics unless this Proxy can still produce valid answers.
//
// Only a released Proxy is rejected. PoolManager.Reset no longer invalidates
// live Proxies: it installs a replacement Pool and retires the old one, which
// stays open until its own scenes release. So a Reset that lands mid-search
// costs nothing — the search finishes on the map it started with.
//
// The closed check is a backstop against an internal bug rather than a case a
// caller can reach: a Pool is closed only once its last scene is gone, and a
// live Proxy holds a reference, so its Pool cannot be closed.
func (p *proxy) checkLive() {
	if p.released.Load() {
		panic("grid: method called on a released Proxy; Acquire a new one")
	}
	if p.pool.Closed() {
		panic("grid: internal error: a live Proxy's Pool was closed")
	}
}

// live returns the resident Base instance, panicking if this Proxy is dead.
//
// Base is cached and re-read only when the Pool's epoch moves, which is what
// Reset bumps. A plain field (not atomic) is enough: a Proxy is single-goroutine
// by contract, so the only shared read is the atomic Epoch load.
func (p *proxy) live() ReadOnlyGrid {
	p.checkLive()
	// Re-read after a Reset, or this Proxy would keep answering from the map it
	// started on. Epoch() is a single atomic load, so the common case is one
	// extra load on top of the released check.
	if epoch := p.pool.Epoch(); p.base == nil || p.baseEpoch != epoch {
		p.base = p.pool.Base()
		p.baseEpoch = epoch
	}
	return p.base
}

// --- pathfinding: leased instance, result copied into the Proxy's own buffer ---

// FindPath searches between two tiles on a leased instance. The result points
// into the Proxy's own buffer and is valid until the next search on this Proxy.
//
// Besides the finder's own results, it reports the Pool's failures: a lease that
// cannot be obtained comes back as ErrPoolTimeout (or ErrPoolClosed), so a busy
// map is distinguishable from an unreachable target.
func (p *proxy) FindPath(x1, y1, x2, y2 int) ([][2]int, error) {
	p.checkLive()
	lease, err := p.pool.Acquire()
	if err != nil {
		return nil, err
	}
	defer lease.Release()

	// The leased instance reuses its buffer on its next search, and another
	// Proxy sharing the Pool can lease this instance at any moment — so copy
	// into our own buffer before letting go of it.
	path, err := lease.FindPath(x1, y1, x2, y2)
	if err != nil {
		return nil, err
	}
	if path == nil {
		return nil, nil
	}
	p.pathBuf = append(p.pathBuf[:0], path...)
	return p.pathBuf, nil
}

// FindPathWorld searches between two world positions on a leased instance. The
// result points into the Proxy's own buffer and is valid until the next search
// on this Proxy. It reports Pool failures the same way FindPath does.
func (p *proxy) FindPathWorld(wx1, wy1, wx2, wy2 float32) ([][2]float32, error) {
	p.checkLive()
	lease, err := p.pool.Acquire()
	if err != nil {
		return nil, err
	}
	defer lease.Release()

	path, err := lease.FindPathWorld(wx1, wy1, wx2, wy2)
	if err != nil {
		return nil, err
	}
	if path == nil {
		return nil, nil
	}
	p.worldBuf = append(p.worldBuf[:0], path...)
	return p.worldBuf, nil
}

// --- everything else: forwarded to the resident Base instance ---
// Proxy is read-only, so these calls cannot race with mutations. Map changes
// go through PoolManager.Reset.

func (p *proxy) Width() int  { return p.live().Width() }
func (p *proxy) Height() int { return p.live().Height() }
func (p *proxy) IsInside(x, y int) bool {
	return p.live().IsInside(x, y)
}

func (p *proxy) IsWalkableAt(x, y int) bool { return p.live().IsWalkableAt(x, y) }

func (p *proxy) GetNodeAt(x, y int) *finder.Node { return p.live().GetNodeAt(x, y) }
func (p *proxy) GetNeighbors(node *finder.Node, diagonal finder.DiagonalMovement, buffer []*finder.Node) []*finder.Node {
	return p.live().GetNeighbors(node, diagonal, buffer)
}

// Clone returns an independent copy of the underlying grid. The copy is detached
// from the Pool and is safe to use (and mutate) without going through a Proxy.
func (p *proxy) Clone() finder.Grid { return p.live().Clone() }

func (p *proxy) TileWidth() int  { return p.live().TileWidth() }
func (p *proxy) TileHeight() int { return p.live().TileHeight() }
func (p *proxy) ObstacleCount() int {
	return p.live().ObstacleCount()
}

func (p *proxy) TileIndex(x, y int) int { return p.live().TileIndex(x, y) }
func (p *proxy) TileXY(index int) (int, int) {
	return p.live().TileXY(index)
}

func (p *proxy) GetWeightAt(x, y int) float64 { return p.live().GetWeightAt(x, y) }

func (p *proxy) WorldToTile(wx, wy float32) (int, int) { return p.live().WorldToTile(wx, wy) }
func (p *proxy) TileToWorld(tx, ty int) (float32, float32) {
	return p.live().TileToWorld(tx, ty)
}
func (p *proxy) IsWalkableAtWorld(wx, wy float32) bool { return p.live().IsWalkableAtWorld(wx, wy) }

func (p *proxy) HasLineOfSight(x1, y1, x2, y2 int) bool {
	return p.live().HasLineOfSight(x1, y1, x2, y2)
}
func (p *proxy) HasLineOfSightWorld(x1, y1, x2, y2 float32) bool {
	return p.live().HasLineOfSightWorld(x1, y1, x2, y2)
}

func (p *proxy) FindNearestWalkable(wx, wy float32, maxRadius int) (int, int, bool) {
	return p.live().FindNearestWalkable(wx, wy, maxRadius)
}
func (p *proxy) FindNearestWalkableWorld(wx, wy float32, maxRadius int, edgeInset float32) (float32, float32, bool) {
	return p.live().FindNearestWalkableWorld(wx, wy, maxRadius, edgeInset)
}

func (p *proxy) Finder() finder.Finder { return p.live().Finder() }

func (p *proxy) RenderSVG(cfg *SVGOpts, paths ...[][2]int) string {
	return p.live().RenderSVG(cfg, paths...)
}

// Release drops this Proxy's scene reference, letting the PoolManager reclaim the
// Pool once the last scene lets go of the same key. Every Grid method panics
// afterwards.
func (p *proxy) Release() {
	p.releaseOnce.Do(func() {
		p.released.Store(true)
		p.manager.release(p.key, p.pool)
		// Drop references so a retained released Proxy cannot keep the map,
		// its Base grid, or result buffers alive.
		p.base = nil
		p.pathBuf = nil
		p.worldBuf = nil
		p.pool = nil
		p.manager = nil
	})
}
