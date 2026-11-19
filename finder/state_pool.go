package finder

import (
	"sync"
	"sync/atomic"
)

// defaultMaxPooledStateNodes bounds how large a state array the pool retains by
// default. Arrays above the limit are dropped after the search instead of being
// pinned in the pool. Tune this for the target grid sizes with
// SetMaxPooledStateNodes.
const defaultMaxPooledStateNodes = 1 << 18

var maxPooledStateNodes atomic.Int64

func init() { maxPooledStateNodes.Store(defaultMaxPooledStateNodes) }

// SetMaxPooledStateNodes sets how many nodes' search state the pool may retain.
// A value <= 0 disables retaining state arrays (the pool still reuses the small
// wrapper object). Larger values avoid reallocating state arrays for large
// grids, at the cost of holding more memory per pooled array.
func SetMaxPooledStateNodes(n int) {
	if n < 0 {
		n = 0
	}
	maxPooledStateNodes.Store(int64(n))
}

var statePool = sync.Pool{
	New: func() any { return new(searchStates) },
}

func acquireStates(width, height int, seq int64) *searchStates {
	s := statePool.Get().(*searchStates)
	s.begin(width, height, seq)
	return s
}

func releaseStates(s *searchStates) {
	if s == nil {
		return
	}
	limit := int(maxPooledStateNodes.Load())
	if limit <= 0 || cap(s.states) > limit {
		s.states = nil
	}
	statePool.Put(s)
}
