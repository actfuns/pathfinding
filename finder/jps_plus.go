package finder

import (
	"encoding/binary"
	"errors"
	"fmt"

	"github.com/actfuns/pathfinding/internal/walkability"
)

// Direction index mapping for JPS+ jump table.
// Each cell stores 8 distances: N, NE, E, SE, S, SW, W, NW.
const (
	dirN  = 0
	dirNE = 1
	dirE  = 2
	dirSE = 3
	dirS  = 4
	dirSW = 5
	dirW  = 6
	dirNW = 7
)

// Jump-table format magics: JPS (int8) and JPS2 (int16).
const (
	jpsPlusMagic8          uint32 = 0x4A5053
	jpsPlusMagic16         uint32 = 0x4A505332
	jpsPlusCanonicalFlag   uint32 = 1 << 31
	jpsPlusFingerprintFlag uint32 = 1 << 29
)

var jpsCardinalDirs = [4][2]int32{
	{0, -1}, // N
	{1, 0},  // E
	{0, 1},  // S
	{-1, 0}, // W
}

var jpsDiagonalDirs = [4][2]int32{
	{1, -1},  // NE
	{1, 1},   // SE
	{-1, 1},  // SW
	{-1, -1}, // NW
}

// jpsCardinalIdx maps cardinal direction vectors to dir index.
var jpsCardinalIdx = map[[2]int32]int{
	{0, -1}: dirN,
	{1, 0}:  dirE,
	{0, 1}:  dirS,
	{-1, 0}: dirW,
}

// jpsDiagonalIdx maps diagonal direction vectors to dir index.
var jpsDiagonalIdx = map[[2]int32]int{
	{1, -1}:  dirNE,
	{1, 1}:   dirSE,
	{-1, 1}:  dirSW,
	{-1, -1}: dirNW,
}

// JPSPlusFinder implements Jump Point Search Plus — a precomputation-based
// optimization of JPS. It replaces the iterative jump() function with an
// O(1) table lookup of precomputed jump distances.
//
// Call Precompute(grid) before FindPath for axis-aligned grids. The grid is
// assumed to be static; changes to the grid require calling Precompute again.
// Precompute returns ErrUnsupportedGrid for hex/staggered grids; FindPath also
// rejects them.
//
// DiagonalAlways uses canonical JPS pruning with goal-dependent jumps resolved
// at query time. Other modes and legacy tables also relax one-step moves.
//
// JPS+ assumes every step has the same cost (tile Weight == 1). Tile weights are
// ignored; use A* or Dijkstra for weighted grids.
//
// JPS+ only supports axis-aligned grids (OrthogonalGrid). Hex and staggered
// grids return ErrUnsupportedGrid.
type JPSPlusFinder struct {
	Heuristic        HeuristicFunc
	DiagonalMovement DiagonalMovement
	Weight           float32
	Adaptive         bool

	width, height int32

	// jump8/jump16 store precomputed jump distances per cell per direction.
	// Layout: (y*width + x)*8 + dir
	// Positive = jump point at that distance; negative = obstacle at abs(distance).
	// int8 is used when both dimensions are <= 128; int16 otherwise.
	jump8          []int8
	jump16         []int16
	wide           bool
	canonical      bool
	fingerprint    uint64
	hasFingerprint bool

	// Pre-allocated direction slice (avoid allocation in hot path)
	dirs []int
}

// NewJPSPlusFinder creates a JPS+ finder with the given options.
func NewJPSPlusFinder(opts ...Option) *JPSPlusFinder {
	opt := ApplyOptions(opts)
	diagonal := resolveDiagonalMovement(opt)
	var dirs []int
	if diagonal == DiagonalNever {
		dirs = []int{dirN, dirE, dirS, dirW}
	} else {
		dirs = []int{dirN, dirNE, dirE, dirSE, dirS, dirSW, dirW, dirNW}
	}
	f := &JPSPlusFinder{
		Heuristic:        Manhattan,
		DiagonalMovement: diagonal,
		Weight:           1,
		Adaptive:         opt.Adaptive,
		dirs:             dirs,
	}
	if opt.Heuristic != nil {
		f.Heuristic = opt.Heuristic
	} else if diagonal != DiagonalNever {
		f.Heuristic = Octile
	}
	if opt.Weight > 0 {
		f.Weight = opt.Weight
	}
	return f
}

// Precompute builds the jump distance table for the given grid.
// Must be called before FindPath. The grid is assumed static.
func (j *JPSPlusFinder) Precompute(grid Grid) error {
	if !supportsJPS(grid) {
		return ErrUnsupportedGrid
	}
	j.precompute(grid, grid.Width() > 128 || grid.Height() > 128)
	return nil
}

// precompute builds the table with an explicit cell width. It is split out so
// tests can compare the int8 and int16 encodings of the same grid.
func (j *JPSPlusFinder) precompute(grid Grid, wide bool) {
	j.width = grid.Width()
	j.height = grid.Height()
	j.wide = wide
	j.fingerprint = gridWalkabilityFingerprint(grid)
	j.hasFingerprint = true
	j.canonical = j.DiagonalMovement == DiagonalAlways && j.width <= 32768 && j.height <= 32768
	size := int(j.width * j.height * 8)
	if j.wide {
		if cap(j.jump16) < size {
			j.jump16 = make([]int16, size)
		} else {
			j.jump16 = j.jump16[:size]
			clear(j.jump16)
		}
		j.jump8 = nil
	} else {
		if cap(j.jump8) < size {
			j.jump8 = make([]int8, size)
		} else {
			j.jump8 = j.jump8[:size]
			clear(j.jump8)
		}
		j.jump16 = nil
	}

	if j.canonical {
		j.computeAlwaysDistances(grid)
		return
	}

	// Phase 1: detect primary jump points (forced neighbors)
	primaryJPs := j.detectPrimaryJumpPoints(grid)

	// Phase 2: compute cardinal and diagonal distances
	j.computeDistances(grid, primaryJPs)
}

func gridWalkabilityFingerprint(grid Grid) uint64 {
	if g, ok := grid.(interface{ WalkabilityFingerprint() uint64 }); ok {
		return g.WalkabilityFingerprint()
	}
	width, height := grid.Width(), grid.Height()
	var fingerprint uint64
	for y := int32(0); y < height; y++ {
		for x := int32(0); x < width; x++ {
			if !grid.IsWalkableAt(x, y) {
				fingerprint = walkability.Toggle(fingerprint, uint64(y)*uint64(width)+uint64(x))
			}
		}
	}
	return fingerprint
}

// detectPrimaryJumpPoints scans each walkable cell for forced neighbors.
// Returns a per-cell bitmask of directions where the cell acts as a jump point.
func (j *JPSPlusFinder) detectPrimaryJumpPoints(grid Grid) []uint8 {
	w, h := j.width, j.height
	primaryJPs := make([]uint8, int(w*h))

	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}

			for _, dir := range jpsCardinalDirs {
				nx, ny := x+dir[0], y+dir[1]
				if !grid.IsInside(nx, ny) || grid.IsWalkableAt(nx, ny) {
					continue
				}

				// Perpendicular to wall direction
				var checkDirs [2][2]int32
				if dir[0] != 0 { // horizontal direction (E/W)
					checkDirs = [2][2]int32{{0, -1}, {0, 1}} // N/S
				} else { // vertical direction (N/S)
					checkDirs = [2][2]int32{{-1, 0}, {1, 0}} // W/E
				}

				for _, cd := range checkDirs {
					// From the wall cell, check perpendicular
					wnx, wny := nx+cd[0], ny+cd[1]
					if !grid.IsInside(wnx, wny) || !grid.IsWalkableAt(wnx, wny) {
						continue
					}

					// Potential jump point from origin in perpendicular direction
					jpx, jpy := x+cd[0], y+cd[1]
					if !grid.IsInside(jpx, jpy) || !grid.IsWalkableAt(jpx, jpy) {
						continue
					}

					dirIdx := jpsCardinalIdx[cd]
					primaryJPs[int(jpy*w+jpx)] |= 1 << dirIdx
				}
			}
		}
	}
	return primaryJPs
}

// computeDistances fills the jumpTable with distances in all 8 directions.
func (j *JPSPlusFinder) computeDistances(grid Grid, primaryJPs []uint8) {
	w, h := j.width, j.height

	// Helper: get distance to obstacle or jump point in a cardinal direction
	scanCardinal := func(x, y int32, dx, dy int32, dirIdx int) int16 {
		dist := int32(0)
		for {
			nx, ny := x+dx*(dist+1), y+dy*(dist+1)
			if !grid.IsInside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
				return int16(-dist)
			}
			if primaryJPs[int(ny*w+nx)]&(1<<dirIdx) != 0 {
				return int16(dist + 1)
			}
			dist++
		}
	}

	// Part A: Cardinal directions
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}
			base := int((y*w + x) * 8)
			for _, dir := range jpsCardinalDirs {
				dirIdx := jpsCardinalIdx[dir]
				j.setTable(base+dirIdx, scanCardinal(x, y, dir[0], dir[1], dirIdx))
			}
		}
	}

	// Part B: Diagonal directions
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}
			base := int((y*w + x) * 8)

			for _, ddir := range jpsDiagonalDirs {
				ddirIdx := jpsDiagonalIdx[ddir]
				hdx, hdy := ddir[0], int32(0) // horizontal component
				vdx, vdy := int32(0), ddir[1] // vertical component
				hDirIdx := jpsCardinalIdx[[2]int32{hdx, hdy}]
				vDirIdx := jpsCardinalIdx[[2]int32{vdx, vdy}]

				dist := int32(0)
				for {
					cx, cy := x+ddir[0]*dist, y+ddir[1]*dist
					if !grid.IsInside(cx, cy) || !grid.IsWalkableAt(cx, cy) {
						j.setTable(base+ddirIdx, int16(-dist))
						break
					}

					if dist > 0 {
						// Check cardinal neighbors at this diagonal position
						cb := int((cy*w + cx) * 8)
						if j.getTable(cb+hDirIdx) > 0 || j.getTable(cb+vDirIdx) > 0 {
							j.setTable(base+ddirIdx, int16(dist))
							break
						}
					}

					// Check horizontal and vertical neighbors are walkable
					hnx, hny := cx+hdx, cy+hdy
					vnx, vny := cx+vdx, cy+vdy
					if !grid.IsInside(hnx, hny) || !grid.IsWalkableAt(hnx, hny) {
						j.setTable(base+ddirIdx, int16(-dist))
						break
					}
					if !grid.IsInside(vnx, vny) || !grid.IsWalkableAt(vnx, vny) {
						j.setTable(base+ddirIdx, int16(-dist))
						break
					}

					// Next diagonal step
					nx, ny := cx+ddir[0], cy+ddir[1]
					if !grid.IsInside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
						j.setTable(base+ddirIdx, int16(-(dist + 1)))
						break
					}
					dist++
				}
			}
		}
	}
}

// hasTable reports whether Precompute (or LoadPrecomputed) has run.
func (j *JPSPlusFinder) hasTable() bool { return j.jump8 != nil || j.jump16 != nil }

func (j *JPSPlusFinder) getTable(i int) int16 {
	if j.wide {
		return j.jump16[i]
	}
	return int16(j.jump8[i])
}

func (j *JPSPlusFinder) setTable(i int, v int16) {
	if j.wide {
		j.jump16[i] = v
	} else {
		j.jump8[i] = int8(v)
	}
}

// getDist returns the precomputed jump distance for cell (x,y) in direction dirIdx.
func (j *JPSPlusFinder) getDist(x, y int32, dirIdx int) int16 {
	if x < 0 || x >= j.width || y < 0 || y >= j.height {
		return 0
	}
	i := int((y*j.width+x)*8) + dirIdx
	if j.wide {
		return j.jump16[i]
	}
	return int16(j.jump8[i])
}

// PrecomputedData serializes the jump table to binary data.
// The returned slice can be saved to a file and loaded later with LoadPrecomputed,
// avoiding the cost of re-running Precompute on server restart.
func (j *JPSPlusFinder) PrecomputedData() ([]byte, error) {
	if !j.hasTable() {
		return nil, errors.New("JPSPlusFinder: Precompute must be called before Export")
	}
	magic := jpsPlusMagic8
	cellBytes := 1
	if j.wide {
		magic = jpsPlusMagic16
		cellBytes = 2
	}
	if j.canonical {
		magic |= jpsPlusCanonicalFlag
	}
	headerSize := 12
	if j.hasFingerprint {
		magic |= jpsPlusFingerprintFlag
		headerSize += 8
	}
	// Format: magic(4) + width(4) + height(4) + optional fingerprint(8) + table.
	buf := make([]byte, j.PrecomputedSize())
	binary.LittleEndian.PutUint32(buf[0:4], magic)
	binary.LittleEndian.PutUint32(buf[4:8], uint32(j.width))
	binary.LittleEndian.PutUint32(buf[8:12], uint32(j.height))
	if j.hasFingerprint {
		binary.LittleEndian.PutUint64(buf[12:20], j.fingerprint)
	}
	if j.wide {
		for i, v := range j.jump16 {
			binary.LittleEndian.PutUint16(buf[headerSize+i*cellBytes:], uint16(v))
		}
	} else {
		for i, v := range j.jump8 {
			buf[headerSize+i] = byte(v)
		}
	}
	return buf, nil
}

// LoadPrecomputed loads previously exported jump table data.
// FindPath verifies the dimensions and obstacle layout against the table.
// Canonical tables also require DiagonalAlways.
// Legacy data without a fingerprint can be loaded, but searches use online
// JPS until Precompute builds a table bound to the map.
func (j *JPSPlusFinder) LoadPrecomputed(data []byte) error {
	if len(data) < 12 {
		return errors.New("JPSPlusFinder: precomputed data too short")
	}
	magic := binary.LittleEndian.Uint32(data[0:4])
	var wide bool
	switch magic &^ (jpsPlusCanonicalFlag | jpsPlusFingerprintFlag) {
	case jpsPlusMagic8:
	case jpsPlusMagic16:
		wide = true
	default:
		return fmt.Errorf("JPSPlusFinder: invalid magic 0x%X", magic)
	}
	w := binary.LittleEndian.Uint32(data[4:8])
	h := binary.LittleEndian.Uint32(data[8:12])
	hasFingerprint := magic&jpsPlusFingerprintFlag != 0
	headerSize := 12
	var fingerprint uint64
	if hasFingerprint {
		headerSize += 8
		if len(data) < headerSize {
			return errors.New("JPSPlusFinder: precomputed data is missing its fingerprint")
		}
		fingerprint = binary.LittleEndian.Uint64(data[12:20])
	}
	cellBytes := 1
	if wide {
		cellBytes = 2
	}
	cells := uint64(w) * uint64(h)
	if w == 0 || h == 0 || w > 1<<31-1 || h > 1<<31-1 || cells > uint64((len(data)-headerSize)/(8*cellBytes)) {
		return errors.New("JPSPlusFinder: invalid dimensions or truncated precomputed data")
	}
	tableSize := int(cells * 8)
	j.width = int32(w)
	j.height = int32(h)
	j.wide = wide
	j.canonical = magic&jpsPlusCanonicalFlag != 0
	j.fingerprint = fingerprint
	j.hasFingerprint = hasFingerprint
	if wide {
		if cap(j.jump16) < tableSize {
			j.jump16 = make([]int16, tableSize)
		} else {
			j.jump16 = j.jump16[:tableSize]
		}
		j.jump8 = nil
		for i := 0; i < tableSize; i++ {
			j.jump16[i] = int16(binary.LittleEndian.Uint16(data[headerSize+i*2:]))
		}
	} else {
		if cap(j.jump8) < tableSize {
			j.jump8 = make([]int8, tableSize)
		} else {
			j.jump8 = j.jump8[:tableSize]
		}
		j.jump16 = nil
		for i := 0; i < tableSize; i++ {
			j.jump8[i] = int8(data[headerSize+i])
		}
	}
	return nil
}

// PrecomputedSize returns the byte size of the serialized jump table.
// Useful for estimating storage or memory before exporting.
func (j *JPSPlusFinder) PrecomputedSize() int {
	if !j.hasTable() {
		return 0
	}
	cellBytes := 1
	if j.wide {
		cellBytes = 2
	}
	headerSize := 12
	if j.hasFingerprint {
		headerSize += 8
	}
	return headerSize + len(j.jump8) + len(j.jump16)*cellBytes
}

// AdaptiveDensityThreshold is the obstacle ratio at or above which Adaptive
// JPS+ prefers A*.
const AdaptiveDensityThreshold = 0.1

func (j *JPSPlusFinder) useAStar(grid Grid) bool {
	oc, ok := grid.(interface{ ObstacleCount() int32 })
	if !ok {
		return false
	}
	area := grid.Width() * grid.Height()
	if area == 0 {
		return false
	}
	return float32(oc.ObstacleCount())/float32(area) >= AdaptiveDensityThreshold
}

func (j *JPSPlusFinder) newAdaptiveFallback() *AStarFinder {
	return NewAStarFinder(
		WithDiagonal(j.DiagonalMovement),
		WithHeuristic(j.Heuristic),
		WithWeight(j.Weight),
	)
}

func (j *JPSPlusFinder) preflight(startX, startY, endX, endY int32, grid Grid) (path [][2]int32, err error) {
	if !grid.IsInside(startX, startY) || !grid.IsInside(endX, endY) {
		return nil, ErrOutOfBounds
	}
	if !supportsJPS(grid) {
		return nil, ErrUnsupportedGrid
	}
	if path, err = checkEndpoints(startX, startY, endX, endY, grid); path != nil || err != nil {
		return path, err
	}
	if !j.hasTable() {
		panic("JPSPlusFinder: Precompute(grid) must be called before FindPath")
	}
	if grid.Width() != j.width || grid.Height() != j.height ||
		(j.canonical && j.DiagonalMovement != DiagonalAlways) ||
		(j.hasFingerprint && gridWalkabilityFingerprint(grid) != j.fingerprint) {
		return nil, ErrPrecomputedMismatch
	}
	return nil, nil
}

// FindPath performs JPS+ pathfinding using the precomputed jump table.
// Precompute(grid) must have been called first with the same grid.
func (j *JPSPlusFinder) FindPath(startX, startY, endX, endY int32, grid Grid, dst [][2]int32) ([][2]int32, error) {
	if path, err := j.preflight(startX, startY, endX, endY, grid); path != nil || err != nil {
		return finishPath(path, dst, true), err
	}
	if j.Adaptive && j.useAStar(grid) {
		return j.newAdaptiveFallback().FindPath(startX, startY, endX, endY, grid, dst)
	}
	if !j.hasFingerprint {
		fallback := NewJumpPointFinder(WithDiagonal(j.DiagonalMovement), WithHeuristic(j.Heuristic), WithWeight(j.Weight))
		return fallback.FindPath(startX, startY, endX, endY, grid, dst)
	}
	searchSeq := globalSearchSeq.Add(1)
	states, err := acquireStates(grid.Width(), grid.Height(), searchSeq)
	if err != nil {
		return nil, err
	}
	defer releaseStates(states)
	if j.canonical {
		p, err := findJPSPath(startX, startY, endX, endY, grid, states, nil, j)
		return finishPath(p, dst, false), err
	}
	p, err := j.findPath(startX, startY, endX, endY, grid, states)
	return finishPath(p, dst, false), err
}

func (j *JPSPlusFinder) findPath(startX, startY, endX, endY int32, grid Grid, states *searchStates) ([][2]int32, error) {
	openList := &minHeap{nodes: states.heap[:0], state: states}
	defer func() {
		clear(openList.nodes[:cap(openList.nodes)])
		states.heap = openList.nodes[:0]
	}()
	pathBuf := states.pathBuf[:0]
	expandBuf := states.expandBuf[:0]
	defer func() {
		states.pathBuf = pathBuf
		states.expandBuf = expandBuf
	}()

	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)
	if startNode == nil || endNode == nil {
		return nil, nil
	}

	startState := states.get(startNode)
	states.get(endNode)
	walk := newWalkableView(grid)
	startState.G = 0
	startState.F = 0
	startIdx := states.index(startNode)
	endIdx := states.index(endNode)
	openList.Push(startIdx, startState.F)

	for !openList.Empty() {
		nodeIdx := openList.Pop()
		node := grid.GetNodeAt(nodeIdx%states.width, nodeIdx/states.width)
		states.states[nodeIdx].Closed = true

		if nodeIdx == endIdx {
			pathBuf = states.backtrace(node, pathBuf)
			expandBuf = expandBuf[:0]
			for i := 0; i < len(pathBuf)-1; i++ {
				x0, y0 := pathBuf[i][0], pathBuf[i][1]
				x1, y1 := pathBuf[i+1][0], pathBuf[i+1][1]
				dx := x1 - x0
				dy := y1 - y0
				var sx, sy int32
				if dx < 0 {
					dx = -dx
					sx = -1
				} else {
					sx = 1
				}
				if dy < 0 {
					dy = -dy
					sy = -1
				} else {
					sy = 1
				}
				err := dx - dy
				for {
					if x0 == x1 && y0 == y1 {
						break
					}
					expandBuf = append(expandBuf, [2]int32{x0, y0})
					e2 := 2 * err
					if e2 > -dy {
						err -= dy
						x0 += sx
					}
					if e2 < dx {
						err += dx
						y0 += sy
					}
				}
			}
			expandBuf = append(expandBuf, pathBuf[len(pathBuf)-1])
			return expandBuf, nil
		}

		j.identifySuccessors(node, endNode, nodeIdx, endIdx, states, openList, &walk)
	}
	return nil, nil
}

func (j *JPSPlusFinder) identifySuccessors(node, endNode *Node, nodeIdx, endIdx int32, states *searchStates, openList *minHeap, walk *walkableView) {
	x, y := node.X, node.Y
	endX, endY := endNode.X, endNode.Y
	heuristic := j.Heuristic
	// The current node was just popped and the end state was initialized before the search.
	nodeG := states.states[nodeIdx].G
	endState := &states.states[endIdx]

	// Use diagonal movement setting to determine which directions to check
	dirs := j.dirs

	// relaxAdjacent offers a one-step move in (dx,dy). It keeps the search
	// complete and optimal when the static jump table cannot encode every
	// goal-dependent jump point.
	relaxAdjacent := func(dx, dy int32) {
		if dx != 0 && dy != 0 && !j.diagonalStepAllowed(walk, x, y, dx, dy) {
			return
		}
		nx, ny := x+dx, y+dy
		if !walk.at(nx, ny) {
			return
		}
		adjIdx := ny*states.width + nx
		ns := states.at(adjIdx)
		if ns.Closed {
			return
		}
		var moveCost float32 = 1
		if dx != 0 && dy != 0 {
			moveCost = SQRT2
		}
		ng := nodeG + moveCost
		if ns.Opened == 0 || ng < ns.G {
			ns.G = ng
			if ns.Opened == 0 {
				ns.H = heuristic(float32(AbsInt(nx-endX)), float32(AbsInt(ny-endY)))
			}
			ns.F = ns.G + ns.H*j.Weight
			ns.Parent = nodeIdx
			if ns.Opened == 0 {
				openList.Push(adjIdx, ns.F)
				ns.Opened = 1
			} else {
				openList.UpdateItem(adjIdx, ns.F)
			}
		}
	}

	for _, dirIdx := range dirs {
		dist := j.getDist(x, y, dirIdx)
		dx, dy := dirToVec(dirIdx)

		// Check if the end node lies in this direction
		if endInDirection(x, y, endX, endY, dx, dy) {
			endDist := absDirDist(endX-x, endY-y, dx, dy)
			obstacleDist := int32(-dist) // positive when no jump point (dist < 0)
			if (dist <= 0 && endDist <= obstacleDist) || (dist > 0 && endDist <= int32(dist)) {
				// End is reachable in this direction, add it as successor
				gd := octileDist(AbsInt(endX-x), AbsInt(endY-y))
				ens := endState
				ng := nodeG + gd
				if !ens.Closed && (ens.Opened == 0 || ng < ens.G) {
					ens.G = ng
					if ens.Opened == 0 {
						ens.H = 0
					}
					ens.F = ng
					ens.Parent = nodeIdx
					if ens.Opened == 0 {
						openList.Push(endIdx, ens.F)
						ens.Opened = 1
					}
				}
				continue
			}
		}

		if dist <= 0 {
			// No jump point in this direction — take a normal step.
			relaxAdjacent(dx, dy)
			continue
		}

		// Positive dist: jump point exists at this distance
		jx, jy := x+dx*int32(dist), y+dy*int32(dist)

		if !walk.at(jx, jy) {
			continue
		}

		jumpIdx := jy*states.width + jx
		ns := states.at(jumpIdx)
		if ns.Closed {
			continue
		}

		gd := octileDist(AbsInt(jx-x), AbsInt(jy-y))
		ng := nodeG + gd

		if ns.Opened == 0 || ng < ns.G {
			ns.G = ng
			if ns.Opened == 0 {
				ns.H = heuristic(float32(AbsInt(jx-endX)), float32(AbsInt(jy-endY)))
			}
			ns.F = ns.G + ns.H*j.Weight
			ns.Parent = nodeIdx

			if ns.Opened == 0 {
				openList.Push(jumpIdx, ns.F)
				ns.Opened = 1
			} else {
				openList.UpdateItem(jumpIdx, ns.F)
			}
		}

		// Also allow turning at the intermediate cell; the static table cannot
		// represent jump points that only exist because of the current goal.
		if dist > 1 {
			relaxAdjacent(dx, dy)
		}
	}
}

// diagonalStepAllowed applies the configured diagonal movement rule to a
// one-step diagonal move from (x, y).
func (j *JPSPlusFinder) diagonalStepAllowed(walk *walkableView, x, y, dx, dy int32) bool {
	switch j.DiagonalMovement {
	case DiagonalNever:
		return false
	case DiagonalOnlyWhenNoObstacles:
		return walk.at(x+dx, y) && walk.at(x, y+dy)
	case DiagonalIfAtMostOneObstacle:
		return walk.at(x+dx, y) || walk.at(x, y+dy)
	default: // DiagonalAlways
		return true
	}
}

// endInDirection checks if (ex,ey) is reachable in direction (dx,dy) from (x,y).
func endInDirection(x, y, ex, ey, dx, dy int32) bool {
	rx, ry := ex-x, ey-y
	if dx == 0 && dy == 0 {
		return false
	}
	if dx == 0 {
		return rx == 0 && ry*dy > 0 && ry%dy == 0
	}
	if dy == 0 {
		return ry == 0 && rx*dx > 0 && rx%dx == 0
	}
	return rx*dx > 0 && ry*dy > 0 && rx/dx == ry/dy && rx%dx == 0 && ry%dy == 0
}

// absDirDist returns the number of steps from (rx,ry) in direction (dx,dy).
func absDirDist(rx, ry, dx, dy int32) int32 {
	if dx != 0 {
		return AbsInt(rx / dx)
	}
	return AbsInt(ry / dy)
}

func octileDist(dx, dy int32) float32 {
	f := SQRT2 - 1
	if dx < dy {
		return f*float32(dx) + float32(dy)
	}
	return f*float32(dy) + float32(dx)
}

func dirToVec(dirIdx int) (int32, int32) {
	switch dirIdx {
	case dirN:
		return 0, -1
	case dirNE:
		return 1, -1
	case dirE:
		return 1, 0
	case dirSE:
		return 1, 1
	case dirS:
		return 0, 1
	case dirSW:
		return -1, 1
	case dirW:
		return -1, 0
	case dirNW:
		return -1, -1
	default:
		return 0, 0
	}
}
