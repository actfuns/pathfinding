package finder

// heapEntry stores the flat state index plus F and H. Keeping the priority
// values in the entry makes heap comparisons independent of the state array.
type heapEntry struct {
	Index int32
	F     float32
	H     float32
}

// minHeap is a binary heap for the A* open list, ordered by F value. When
// tieBreak is set, equal F values are ordered by H using the per-search state.
type minHeap struct {
	nodes    []heapEntry
	tieBreak bool
	state    *searchStates
}

func (h *minHeap) less(a, b heapEntry) bool {
	if a.F != b.F {
		return a.F < b.F
	}
	if h.tieBreak {
		return a.H < b.H
	}
	return false
}

func (h *minHeap) Len() int { return len(h.nodes) }

func (h *minHeap) Empty() bool { return len(h.nodes) == 0 }

func (h *minHeap) Push(index int32, f float32) {
	pos := len(h.nodes)
	h.nodes = append(h.nodes, heapEntry{Index: index, F: f, H: h.state.states[index].H})
	h.setHeapIndex(index, int32(pos))
	h.siftUp(pos)
}

func (h *minHeap) Pop() int32 {
	n := len(h.nodes)
	root := h.nodes[0]
	last := h.nodes[n-1]
	h.nodes = h.nodes[:n-1]
	h.setHeapIndex(root.Index, -1)
	if n == 1 {
		return root.Index
	}
	h.nodes[0] = last
	h.setHeapIndex(last.Index, 0)
	h.siftDown(0)
	return root.Index
}

func (h *minHeap) UpdateItem(index int32, f float32) {
	pos := int(h.state.states[index].HeapIndex)
	if pos < 0 || pos >= len(h.nodes) || h.nodes[pos].Index != index {
		return
	}
	h.nodes[pos].F = f
	h.nodes[pos].H = h.state.states[index].H
	h.siftUp(pos)
}

func (h *minHeap) siftUp(pos int) {
	entry := h.nodes[pos]
	for pos > 0 {
		parent := (pos - 1) >> 1
		parentEntry := h.nodes[parent]
		if !h.less(entry, parentEntry) {
			break
		}
		h.nodes[pos] = parentEntry
		h.setHeapIndex(parentEntry.Index, int32(pos))
		pos = parent
	}
	h.nodes[pos] = entry
	h.setHeapIndex(entry.Index, int32(pos))
}

func (h *minHeap) siftDown(pos int) {
	n := len(h.nodes)
	entry := h.nodes[pos]
	for {
		child := pos*2 + 1
		if child >= n {
			break
		}
		right := child + 1
		if right < n && h.less(h.nodes[right], h.nodes[child]) {
			child = right
		}
		if !h.less(h.nodes[child], entry) {
			break
		}
		childEntry := h.nodes[child]
		h.nodes[pos] = childEntry
		h.setHeapIndex(childEntry.Index, int32(pos))
		pos = child
	}
	h.nodes[pos] = entry
	h.setHeapIndex(entry.Index, int32(pos))
}

func (h *minHeap) setHeapIndex(index int32, pos int32) {
	h.state.states[index].HeapIndex = pos
}
