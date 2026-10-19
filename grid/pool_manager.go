package grid

import "sync"

// PoolManager owns one Pool per map key and recycles a Pool once the last scene
// using that key is gone.
//
// Locks are always taken manager-then-pool, and a Pool is closed only after the
// manager lock is dropped, so closing one map never blocks another.
//
// All methods are safe for concurrent use.
type PoolManager struct {
	mu    sync.Mutex
	pools map[string]*Pool
}

// NewPoolManager creates an empty PoolManager. Pools are created on demand by
// Acquire, which is where their capacity is set: capacity is a per-map concern,
// since how many searches one map has to serve at once has little to do with the
// others.
func NewPoolManager() *PoolManager {
	return &PoolManager{pools: make(map[string]*Pool)}
}

// Acquire returns the Proxy a scene uses to work with the map named key,
// creating the map's Pool on first use and taking one reference on it.
//
// factory builds the Pool's instances; it is only used when the Pool is
// created. Options tune the Pool and all have defaults — see [WithCapacity] and
// [WithAcquireTimeout]. A later Acquire for a key that already has a Pool joins
// it and ignores both, so they have to stay the same for a given key. Call Reset
// to switch a key over to a new Factory or configuration.
//
// The returned Proxy holds a scene reference: Release it exactly once when the
// scene is done, which is what lets the PoolManager reclaim the Pool.
//
// Mutating a Proxy through SetWalkableAt/SetWeightAt affects only the instance
// backing its read-only calls — pathfinding runs on other instances created by
// factory, which do not observe those changes. See [Factory].
func (m *PoolManager) Acquire(key string, factory Factory, opts ...PoolOption) (Proxy, error) {
	m.mu.Lock()
	p := m.pools[key]
	if p == nil {
		var err error
		p, err = newPool(factory, opts...)
		if err != nil {
			m.mu.Unlock()
			return nil, err
		}
		m.pools[key] = p
	}
	p.addRef()
	m.mu.Unlock()

	return &proxy{pool: p, manager: m, key: key}, nil
}

// release drops a scene reference and, when it was the last one, removes the
// Pool from the PoolManager and closes it.
func (m *PoolManager) release(key string, p *Pool) {
	m.mu.Lock()
	if m.pools[key] != p {
		m.mu.Unlock()
		return
	}
	if !p.releaseRef() {
		m.mu.Unlock()
		return
	}
	delete(m.pools, key)
	m.mu.Unlock()

	// Close without the manager lock so other maps stay responsive.
	p.close()
}

// Pool returns the Pool registered for key, or nil when no scene holds one.
// It exists for status queries and inspection; the Pool it returns is not
// leased to the caller.
func (m *PoolManager) Pool(key string) *Pool {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.pools[key]
}

// Reset swaps in a new map for key while keeping the Pool that serves it: the
// scene references, the resident Base instance and every Proxy already handed out
// all stay valid, and nothing panics. Use it when the map data behind key changes.
//
// Internally the Pool takes the new Factory and bumps an epoch. Instances built
// from the old Factory become stale and are discarded — idle ones immediately,
// ones on lease when their search finishes — so the next lease builds from the
// new data. Searches already running finish against the map they started with.
//
// Capacity and acquire timeout belong to the Pool, not to the map, so Reset
// leaves them alone. Pass different options to Acquire for a new key instead.
//
//	pr, _ := mgr.Acquire(key, factory, WithCapacity(4))
//	// ... map data changed ...
//	mgr.Reset(key, newFactory)
//	// pr keeps working, now against the new map
//
// It returns ErrNilFactory if factory is nil, and ErrNoPool if no scene holds
// key, so a failed swap is never mistaken for a successful one.
func (m *PoolManager) Reset(key string, factory Factory) error {
	if factory == nil {
		return ErrNilFactory
	}
	m.mu.Lock()
	p := m.pools[key]
	m.mu.Unlock()
	if p == nil {
		return ErrNoPool
	}
	p.reset(factory)
	return nil
}
