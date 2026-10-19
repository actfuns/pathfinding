package grid

import (
	"errors"
	"sync"
	"sync/atomic"
	"time"
)

var (
	// ErrPoolClosed is returned by Acquire when the Pool has been closed.
	ErrPoolClosed = errors.New("grid pool is closed")
	// ErrFactoryNil is returned by Acquire when the Factory produced a nil Grid.
	ErrFactoryNil = errors.New("grid pool factory returned nil")
	// ErrNilFactory is returned by PoolManager.Acquire and Reset when no Factory
	// was supplied. There is nothing to fall back to, so a Pool cannot be built.
	ErrNilFactory = errors.New("grid pool requires a Factory")
	// ErrNoPool is returned by PoolManager.Reset when no scene holds the key, so
	// there is nothing to replace. Reporting it matters: a silent no-op would
	// look like a successful swap, and the next Acquire would then build the map
	// from the caller's stale Factory.
	ErrNoPool = errors.New("grid pool is not registered for that key")
	// ErrPoolTimeout is returned by Acquire when no instance became available
	// within the Pool's acquire timeout, so the search was never attempted. It
	// is transient — the map may well have a route — so retrying is the right
	// response, unlike a nil path with a nil error.
	ErrPoolTimeout = errors.New("grid pool acquire timed out")
)

// Factory creates a Grid instance for a Pool.
//
// The returned Grid must be fully independent from every other instance the
// same Factory produces: it needs its own tile storage and its own Finder,
// because a search mutates node state (G, F, Parent, Closed, ...) and reuses
// internal buffers. Two instances sharing either would corrupt each other when
// leased concurrently.
//
// Build instances by calling the grid constructor afresh. Do NOT implement this
// as Grid.Clone(): Clone copies the node slice but shares the Finder, and its
// reusable search buffers make two clones unsafe to search concurrently.
//
// A Pool may call the Factory from several goroutines at once, since it builds
// new instances outside its lock, so Factory must be safe for concurrent use.
// It should not panic: a panic is handed back to the caller and costs the Pool
// the reserved capacity slot.
//
// The consequence is that instances are not kept in sync: mutating one Grid
// through SetWalkableAt/SetWeightAt does not affect the others.
type Factory func() Grid

// Lease is exclusive, temporary ownership of one Grid instance held by a Pool.
//
// Every method promoted from Grid may be used without locking for as long as
// the Lease is held. Release must be called exactly once when done; calling it
// again is safe and does nothing. Using the Grid after Release is a bug — the
// instance may already be leased to somebody else.
//
// Do not hold a Lease across a barrier that the holder of another Lease has to
// reach before releasing it. Two ways this deadlocks:
//
//   - releasing in one goroutine while another blocks in Pool.Acquire
//   - waiting on anything before releasing, if that thing needs another instance
//
// In both cases the blocked waiter never gets an instance, because instances
// only come back after Release. Acquire is bounded by the Pool's timeout (see
// WithAcquireTimeout), so the damage is capped rather than permanent — but
// release before you block anyway. Proxy.FindPath is shaped correctly: it
// releases via defer before returning to its caller.
type Lease struct {
	Grid

	pool  *Pool
	epoch uint64 // map version this instance was built from
	once  sync.Once
}

// Release returns the instance to its Pool. Extra calls are no-ops.
func (l *Lease) Release() {
	if l == nil {
		return
	}
	l.once.Do(func() { l.pool.releaseInstance(l.Grid, l.epoch) })
}

// Pool keeps a bounded set of interchangeable Grid instances for one map and
// lends them out one at a time, so that concurrent searches never share mutable
// finder state.
//
// Semantics:
//   - Acquire reuses an idle instance; if none is free and the pool is below
//     capacity it builds a new one; otherwise it waits for one to come back, but
//     no longer than the acquire timeout, and reports ErrPoolTimeout after that.
//     It is also woken when the pool closes.
//   - releaseInstance returns an instance to the idle set, unless the pool is
//     closed or holds more instances than there are active scenes, in which case
//     the extra instance is dropped so the pool shrinks back.
//   - refCount tracks live scenes; PoolManager owns it, and closes the pool once
//     count reaches zero.
//
// A separate, always-resident Base instance serves read-only calls so that reads
// never compete for a lease. It is created lazily and is not counted in
// capacity.
//
// All methods are safe for concurrent use.
type Pool struct {
	factory  Factory
	capacity int
	// Budget Acquire waits for before ErrPoolTimeout. <= 0 means wait forever.
	timeout time.Duration

	// Bumped by reset. Instances built under an older epoch belong to the
	// replaced map and are discarded instead of being pooled again.
	epoch atomic.Uint64

	mu        sync.Mutex
	cond      *sync.Cond
	idle      []Grid
	created   int
	refCount  int
	closed    atomic.Bool
	base      Grid   // mu-guarded
	baseEpoch uint64 // mu-guarded: epoch p.base was built from

	// Contention counters, guarded by mu. Updated only when a caller actually
	// has to wait, so the uncontended path pays nothing for them.
	waits        int64
	waitNanos    int64
	maxWaitNanos int64
}

// PoolStats is a snapshot of one Pool's occupancy and contention.
//
// The wait fields are cumulative since the Pool was created, so a caller can
// compare consecutive snapshots to measure a single frame, or read MaxWait as a
// high-water mark to answer "has this map ever made a caller stall badly?".
// A Pool that never reports Waits was never at capacity.
type PoolStats struct {
	// Instances currently existing, leased or idle, excluding Base.
	Instances int
	// Instances currently free to lease.
	Idle int
	// Configured ceiling on concurrent searches.
	Capacity int
	// Scenes currently holding this Pool.
	Scenes int

	// Acquire calls that had to wait for an instance to come back.
	Waits int64
	// Total time callers spent blocked, summed.
	WaitTotal time.Duration
	// Longest single block.
	MaxWait time.Duration
}

// Stats returns a snapshot of the Pool's current occupancy and how much callers
// have had to wait for an instance. Safe to call from any goroutine, including
// from a game loop that must not stall.
func (p *Pool) Stats() PoolStats {
	p.mu.Lock()
	defer p.mu.Unlock()
	return PoolStats{
		Instances: p.created,
		Idle:      len(p.idle),
		Capacity:  p.capacity,
		Scenes:    p.refCount,
		Waits:     p.waits,
		WaitTotal: time.Duration(p.waitNanos),
		MaxWait:   time.Duration(p.maxWaitNanos),
	}
}

// newPool creates a Pool from its options. Options are read here and only here,
// so a Pool's configuration is fixed for its lifetime and needs no locking to
// read.
//
// It returns ErrNilFactory if factory is nil, since there is nothing sensible to
// fall back to.
func newPool(factory Factory, opts ...PoolOption) (*Pool, error) {
	if factory == nil {
		return nil, ErrNilFactory
	}
	cfg := poolOptions{acquireTimeout: DefaultAcquireTimeout}
	for _, opt := range opts {
		opt(&cfg)
	}
	capacity := cfg.capacity
	if capacity < 1 {
		capacity = defaultCapacity()
	}
	p := &Pool{factory: factory, capacity: capacity, timeout: cfg.acquireTimeout}
	p.cond = sync.NewCond(&p.mu)
	return p, nil
}

// Timeout reports the budget Acquire waits for before giving up. Zero or less
// means Acquire waits indefinitely.
//
// It is named for what it reports rather than for the option that sets it
// (WithAcquireTimeout), so it cannot be confused with the act of acquiring.
func (p *Pool) Timeout() time.Duration { return p.timeout }

// Base returns the resident instance used for read-only calls. It is built on
// first use and reused until reset replaces the map, at which point the next
// call rebuilds it. Base is not leased and does not count towards capacity.
func (p *Pool) Base() Grid {
	p.mu.Lock()
	if p.base != nil && p.baseEpoch == p.epoch.Load() {
		b := p.base
		p.mu.Unlock()
		return b
	}
	f, epoch := p.factory, p.epoch.Load()
	p.mu.Unlock()

	b := f() // build outside the lock: the Factory may be slow

	p.mu.Lock()
	if p.base == nil || p.baseEpoch != epoch {
		p.base, p.baseEpoch = b, epoch
	}
	b = p.base
	p.mu.Unlock()
	return b
}

// Epoch reports which map version this Pool is currently serving. Callers that
// cache Base use it to notice when reset has replaced the map underneath them.
func (p *Pool) Epoch() uint64 { return p.epoch.Load() }

// waitForever is the internal budget meaning "block until an instance frees up
// or the pool closes". Callers reach it only by configuring a non-positive
// timeout explicitly.
const waitForever = time.Duration(-1)

// Acquire lends out one instance exclusively.
//
// It reuses an idle instance when possible, otherwise builds a new one while
// below capacity, and otherwise waits for an instance to come back — but never
// past the Pool's acquire timeout (see WithAcquireTimeout, default
// DefaultAcquireTimeout), after which it returns ErrPoolTimeout.
//
// The caller owns the result and must Release it. Because a search goes through
// this path, a Pool that is at capacity fails visibly instead of stalling the
// caller indefinitely.
func (p *Pool) Acquire() (*Lease, error) {
	if p.timeout > 0 {
		return p.acquire(p.timeout)
	}
	return p.acquire(waitForever)
}

// acquire lends an instance, waiting at most budget. budget == waitForever waits
// indefinitely.
func (p *Pool) acquire(budget time.Duration) (lease *Lease, err error) {
	// Set only once this caller has actually had to block, so an uncontended
	// Acquire costs no clock reads. Read by the deferred recorder below, on
	// this goroutine.
	var blockedAt time.Time
	defer func() {
		if blockedAt.IsZero() {
			return
		}
		d := int64(time.Since(blockedAt))
		p.mu.Lock()
		p.waits++
		p.waitNanos += d
		if d > p.maxWaitNanos {
			p.maxWaitNanos = d
		}
		p.mu.Unlock()
	}()

	// sync.Cond cannot time out on its own, so a timer broadcasts at the deadline
	// and every waiter re-checks the clock itself — the broadcast is a nudge, not
	// the decision. The timer is armed lazily, right before the first wait: arming
	// it up front charged every Acquire for a timer, including the common ones
	// that are served from the idle set without ever blocking.
	var deadline time.Time
	if budget > 0 {
		deadline = time.Now().Add(budget)
	}
	var timer *time.Timer
	defer func() {
		if timer != nil {
			timer.Stop()
		}
	}()

	p.mu.Lock()
	for {
		if p.closed.Load() {
			p.mu.Unlock()
			return nil, ErrPoolClosed
		}

		epoch := p.epoch.Load()

		if n := len(p.idle); n > 0 {
			g := p.idle[n-1]
			p.idle = p.idle[:n-1]
			p.mu.Unlock()
			return &Lease{Grid: g, pool: p, epoch: epoch}, nil
		}

		if p.created < p.capacity {
			p.created++          // reserve the slot, then build outside the lock
			factory := p.factory // and read it under the lock: reset rewrites it
			p.mu.Unlock()

			g := p.build(factory)
			if g == nil {
				p.mu.Lock()
				p.created--
				p.cond.Broadcast()
				p.mu.Unlock()
				return nil, ErrFactoryNil
			}
			return &Lease{Grid: g, pool: p, epoch: epoch}, nil
		}

		if !deadline.IsZero() && !time.Now().Before(deadline) {
			p.mu.Unlock()
			return nil, ErrPoolTimeout
		}
		if blockedAt.IsZero() {
			blockedAt = time.Now()
			if !deadline.IsZero() {
				timer = time.AfterFunc(time.Until(deadline), func() {
					p.mu.Lock()
					p.cond.Broadcast()
					p.mu.Unlock()
				})
			}
		}
		p.cond.Wait()
	}
}

// build creates an instance outside the lock, from a Factory captured while the
// lock was held. If the Factory panics the reserved slot is handed back before
// the panic continues, so a bad Factory cannot leak capacity and deadlock the
// pool; the panic still reaches the caller.
func (p *Pool) build(factory Factory) (g Grid) {
	defer func() {
		if r := recover(); r != nil {
			p.mu.Lock()
			p.created--
			p.cond.Broadcast()
			p.mu.Unlock()
			panic(r)
		}
	}()
	return factory()
}

// reset swaps in a new map while keeping this Pool, its scene references and
// every Proxy that already holds it. Nothing is invalidated and nothing panics.
//
// The epoch bump makes every instance built from the old Factory stale: idle
// ones are dropped immediately, and ones still on lease are dropped when they
// come back. Searches already running finish on the map they started with; the
// next lease builds from the new Factory.
func (p *Pool) reset(factory Factory) {
	if factory == nil {
		return
	}
	p.mu.Lock()
	p.factory = factory
	p.epoch.Add(1)
	p.created -= len(p.idle)
	p.idle = nil
	p.base = nil // rebuilt on next use, from the new Factory
	p.cond.Broadcast()
	p.mu.Unlock()
}

// releaseInstance takes a leased instance back. epoch is the map version the
// instance was built from; if the map has been replaced since, the instance is
// stale and is dropped rather than pooled.
func (p *Pool) releaseInstance(g Grid, epoch uint64) {
	p.mu.Lock()
	defer p.mu.Unlock()

	// Closed, built from a replaced map, or more instances than live scenes:
	// drop it, either because it is useless now or because the pool shrinks.
	if p.closed.Load() || epoch != p.epoch.Load() || p.created > p.refCount {
		p.created--
		p.cond.Broadcast() // slot freed, wake waiters so they can build
		return
	}

	p.idle = append(p.idle, g)
	p.cond.Signal()
}

// addRef records one more live scene.
func (p *Pool) addRef() {
	p.mu.Lock()
	p.refCount++
	p.mu.Unlock()
}

// releaseRef drops one live scene and discards idle instances that are no longer
// needed. It reports whether no scenes remain.
func (p *Pool) releaseRef() bool {
	p.mu.Lock()
	defer p.mu.Unlock()

	p.refCount--
	if p.refCount < 0 {
		p.refCount = 0
	}
	for len(p.idle) > 0 && p.created > p.refCount {
		p.created--
		p.idle = p.idle[:len(p.idle)-1]
	}
	p.cond.Broadcast()
	return p.refCount == 0
}

// close marks the pool dead and drops its idle instances. Instances still on
// lease are discarded when they come back.
//
// Only release calls this, and only once the last scene holding the Pool has
// gone. A live Proxy always holds a reference, so a Pool that still has scenes
// is never closed.
func (p *Pool) close() {
	p.mu.Lock()
	defer p.mu.Unlock()

	if !p.closed.CompareAndSwap(false, true) {
		return
	}
	p.created -= len(p.idle)
	p.idle = nil
	p.cond.Broadcast()
}

// Closed reports whether the Pool has been closed, either by PoolManager.Reset or
// by the last scene letting go of it. It is safe to call from any goroutine.
func (p *Pool) Closed() bool { return p.closed.Load() }

// Created returns how many instances currently exist, leased or idle,
// excluding Base.
func (p *Pool) Created() int {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.created
}

// Idle returns how many instances are currently free to lease.
func (p *Pool) Idle() int {
	p.mu.Lock()
	defer p.mu.Unlock()
	return len(p.idle)
}
