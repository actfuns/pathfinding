package finder

import "sync/atomic"

var globalSearchSeq atomic.Int64

// newSearchSeq returns a globally unique search sequence number.
// Each finder instance gets its own base value, so multiple finders can safely
// share the same grid without Node.SearchID collisions.
func newSearchSeq() int {
	return int(globalSearchSeq.Add(1))
}
