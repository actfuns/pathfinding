package finder

// nodeState is the per-search state that used to live inside Node. Parent and
// heap positions are flat tile indices so the state array stays compact and
// does not retain grid nodes.
type nodeState struct {
	SearchID    int64
	G, H, F     float64
	Parent      int32
	HeapIndex   int32
	RetainCount int16
	Opened      int8
	Closed      bool
	Tested      bool
}

// searchStates is a reusable, generation-tagged state array indexed by the flat
// tile index (Y*width + X).
type searchStates struct {
	width, height int
	seq           int64
	states        []nodeState
}

func (s *searchStates) begin(width, height int, seq int64) {
	s.width, s.height, s.seq = width, height, seq
	n := width * height
	if n < 1 {
		n = 1
	}
	if cap(s.states) < n {
		s.states = make([]nodeState, n)
	} else {
		s.states = s.states[:n]
	}
}

func (s *searchStates) get(n *Node) *nodeState {
	return s.at(s.index(n))
}

func (s *searchStates) at(i int32) *nodeState {
	st := &s.states[int(i)]
	if st.SearchID != s.seq {
		*st = nodeState{SearchID: s.seq, Parent: -1, HeapIndex: -1}
	}
	return st
}

func (s *searchStates) index(n *Node) int32 {
	return int32(n.Y*s.width + n.X)
}

func (s *searchStates) xy(i int32) (int, int) {
	return int(i) % s.width, int(i) / s.width
}

// backtrace writes the path from start to node into buf (reversed internally)
// using parent indices.
func (s *searchStates) backtrace(node *Node, buf [][2]int) [][2]int {
	buf = buf[:0]
	for i := s.index(node); i >= 0; {
		x, y := s.xy(i)
		buf = append(buf, [2]int{x, y})
		i = s.at(i).Parent
	}
	for i, j := 0, len(buf)-1; i < j; i, j = i+1, j-1 {
		buf[i], buf[j] = buf[j], buf[i]
	}
	return buf
}

func (s *searchStates) biBacktrace(nodeA, nodeB *Node, buf [][2]int) [][2]int {
	buf = buf[:0]
	var pathA [][2]int
	for i := s.index(nodeA); i >= 0; i = s.at(i).Parent {
		x, y := s.xy(i)
		pathA = append(pathA, [2]int{x, y})
	}
	for i, j := 0, len(pathA)-1; i < j; i, j = i+1, j-1 {
		pathA[i], pathA[j] = pathA[j], pathA[i]
	}
	buf = append(buf, pathA...)
	for i := s.index(nodeB); i >= 0; i = s.at(i).Parent {
		x, y := s.xy(i)
		buf = append(buf, [2]int{x, y})
	}
	return buf
}
