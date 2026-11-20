package finder

import (
	"runtime"
	"sync"
	"time"
)

// boundedPool lazily creates values up to max and blocks when all are in use.
// max is a ceiling, not a preallocation.
type boundedPool[T any] struct {
	mu      sync.Mutex
	cond    *sync.Cond
	idle    []T
	created int
	max     int
	wait    time.Duration
	newFn   func() T
}

func newBoundedPool[T any](max int, newFn func() T) *boundedPool[T] {
	if max < 1 {
		max = 1
	}
	p := &boundedPool[T]{max: max, newFn: newFn}
	p.cond = sync.NewCond(&p.mu)
	return p
}

func (p *boundedPool[T]) acquire() (T, bool) {
	p.mu.Lock()

	// Fast path: reuse an idle slot.
	if n := len(p.idle); n > 0 {
		v := p.idle[n-1]
		p.idle = p.idle[:n-1]
		p.mu.Unlock()
		return v, true
	}

	// Fast path: create a slot if below the ceiling.
	if p.created < p.max {
		v := p.newFn()
		p.created++
		p.mu.Unlock()
		return v, true
	}

	// Slow path: all slots are busy. Only here do we set up a timer.
	if p.wait <= 0 {
		p.mu.Unlock()
		var zero T
		return zero, false
	}
	deadline := time.Now().Add(p.wait)
	timer := time.AfterFunc(p.wait, func() {
		p.mu.Lock()
		p.cond.Broadcast()
		p.mu.Unlock()
	})
	defer timer.Stop()

	for len(p.idle) == 0 && p.created >= p.max {
		if time.Now().After(deadline) {
			p.mu.Unlock()
			var zero T
			return zero, false
		}
		p.cond.Wait()
	}

	var v T
	if n := len(p.idle); n > 0 {
		v = p.idle[n-1]
		p.idle = p.idle[:n-1]
	} else {
		v = p.newFn()
		p.created++
	}
	p.mu.Unlock()
	return v, true
}

func (p *boundedPool[T]) release(v T) {
	p.mu.Lock()
	if len(p.idle) < p.max {
		p.idle = append(p.idle, v)
	} else {
		p.created--
	}
	p.cond.Signal()
	p.mu.Unlock()
}

func (p *boundedPool[T]) setWait(d time.Duration) {
	if d < 0 {
		d = 0
	}
	p.mu.Lock()
	p.wait = d
	p.cond.Broadcast()
	p.mu.Unlock()
}

func (p *boundedPool[T]) setMax(max int) {
	if max < 1 {
		max = 1
	}
	p.mu.Lock()
	p.max = max
	if len(p.idle) > max {
		drop := len(p.idle) - max
		p.idle = p.idle[:max]
		p.created -= drop
	}
	p.cond.Broadcast()
	p.mu.Unlock()
}

func newSearchStates() *searchStates {
	return &searchStates{
		neighborBuf:    make([]*Node, 0, 8),
		neighborPosBuf: make([][2]int32, 0, 8),
	}
}

// DefaultSearchWait is the default maximum time FindPath waits for a free slot.
const DefaultSearchWait = time.Second

var statePool = newBoundedPool(runtime.GOMAXPROCS(0), newSearchStates)

func init() { statePool.setWait(DefaultSearchWait) }

// SetMaxSearchWait sets how long FindPath waits for a free slot before
// returning ErrSearchBusy. A value <= 0 makes it non-blocking.
func SetMaxSearchWait(d time.Duration) {
	statePool.setWait(d)
}

// SetMaxConcurrentSearches sets the upper bound on concurrent FindPath calls.
// The default is runtime.GOMAXPROCS(0). Values below 1 are clamped to 1.
// Lowering the limit does not cancel running searches; idle slots above the new
// limit are released. Raising it lets waiters create new slots immediately.
func SetMaxConcurrentSearches(n int) {
	if n < 1 {
		n = 1
	}
	statePool.setMax(n)
}

func acquireStates(width, height int32, seq int64) (*searchStates, error) {
	s, ok := statePool.acquire()
	if !ok {
		return nil, ErrSearchBusy
	}
	s.begin(width, height, seq)
	return s, nil
}

func releaseStates(s *searchStates) {
	if s == nil {
		return
	}
	clear(s.neighborBuf[:cap(s.neighborBuf)])
	clear(s.queue[:cap(s.queue)])
	clear(s.queueB[:cap(s.queueB)])
	for i := range s.neighborBufs {
		buf := s.neighborBufs[i]
		clear(buf[:cap(buf)])
		s.neighborBufs[i] = buf[:0]
	}
	statePool.release(s)
}

func clonePath(path [][2]int32) [][2]int32 {
	if path == nil {
		return nil
	}
	out := make([][2]int32, len(path))
	copy(out, path)
	return out
}
