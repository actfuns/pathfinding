package grid

import (
	"runtime"
	"time"
)

// DefaultAcquireTimeout bounds how long [Pool.Acquire] waits for an instance
// when the Pool does not say otherwise.
//
// It is deliberately generous. A slow search is not a reason to give up — a
// 500×500 A* search measures around 3 ms, so this leaves roughly 300× headroom,
// and callers routinely queue behind several searches before an instance frees
// up. The bound exists to turn one pathological case into a slow one: a lease
// that is never released (a caller that forgot, or a goroutine that panicked
// outside the usual defer) would otherwise block every later caller forever.
//
// Callers that run on a frame budget should set a tighter bound with
// [WithAcquireTimeout] rather than rely on this one.
const DefaultAcquireTimeout = time.Second

// PoolOption tunes a Pool. Every option is read once, when the Pool is created;
// a Pool's configuration is fixed for its lifetime.
//
// The Factory is not an option: a Pool cannot exist without one, so it is a
// parameter of [PoolManager.Acquire] rather than something with a default.
// Everything tunable lives here, and every option has one.
type PoolOption func(*poolOptions)

type poolOptions struct {
	capacity       int
	acquireTimeout time.Duration
}

// WithCapacity sets how many searches the map may run at once.
//
// Zero or less (the default) means one instance per logical CPU, so a map can
// use every core without anyone having to know how many that is.
//
// Size it to the busiest the map ever gets rather than to its average: too low
// and searches queue behind each other. Capacity costs nothing while idle,
// because a Pool shrinks back to the number of live scenes as soon as a burst of
// searches ends — but each extra instance is a whole extra copy of the map.
func WithCapacity(n int) PoolOption {
	return func(o *poolOptions) { o.capacity = n }
}

// defaultCapacity resolves an unset capacity to the logical CPU count, so an
// uncontended map can use every core.
func defaultCapacity() int { return runtime.NumCPU() }

// WithAcquireTimeout bounds how long [Pool.Acquire] waits for an instance
// before giving up with [ErrPoolTimeout].
//
// Zero or a negative value means wait indefinitely, which restores unbounded
// blocking — reachable only by asking for it explicitly.
//
// The default is [DefaultAcquireTimeout]. Tighten this when a caller must not
// stall; Stats reports whether a Pool ever hits the bound.
func WithAcquireTimeout(d time.Duration) PoolOption {
	return func(o *poolOptions) { o.acquireTimeout = d }
}
