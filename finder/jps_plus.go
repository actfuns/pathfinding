package finder

import (
	"encoding/binary"
	"errors"
	"fmt"
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

var jpsCardinalDirs = [4][2]int{
	{0, -1}, // N
	{1, 0},  // E
	{0, 1},  // S
	{-1, 0}, // W
}

var jpsDiagonalDirs = [4][2]int{
	{1, -1},  // NE
	{1, 1},   // SE
	{-1, 1},  // SW
	{-1, -1}, // NW
}

// jpsCardinalIdx maps cardinal direction vectors to dir index.
var jpsCardinalIdx = map[[2]int]int{
	{0, -1}: dirN,
	{1, 0}:  dirE,
	{0, 1}:  dirS,
	{-1, 0}: dirW,
}

// jpsDiagonalIdx maps diagonal direction vectors to dir index.
var jpsDiagonalIdx = map[[2]int]int{
	{1, -1}:  dirNE,
	{1, 1}:   dirSE,
	{-1, 1}:  dirSW,
	{-1, -1}: dirNW,
}

// JPSPlusFinder implements Jump Point Search Plus — a precomputation-based
// optimization of JPS. It replaces the iterative jump() function with an
// O(1) table lookup of precomputed jump distances.
//
// Call Precompute(grid) before FindPath. The grid is assumed to be static;
// changes to the grid require calling Precompute again.
type JPSPlusFinder struct {
	Heuristic        HeuristicFunc
	DiagonalMovement DiagonalMovement
	Weight           float64

	width, height int

	// jumpTable stores precomputed jump distances per cell per direction.
	// Layout: jumpTable[(y*width + x)*8 + dir]
	// Positive = jump point at that distance; negative = obstacle at abs(distance).
	jumpTable []int8

	// Runtime state
	grid      Grid
	startNode *Node
	endNode   *Node
	openList  *MinHeap
	heapSlice []*Node
	pathBuf   [][2]int
	searchSeq int

	// Pre-allocated direction slice (avoid allocation in hot path)
	dirs []int
}

// NewJPSPlusFinder creates a JPS+ finder with the given options.
func NewJPSPlusFinder(opts ...Option) *JPSPlusFinder {
	opt := ApplyOptions(opts)
	var dirs []int
	if opt.DiagonalMovement == DiagonalNever {
		dirs = []int{dirN, dirE, dirS, dirW}
	} else {
		dirs = []int{dirN, dirNE, dirE, dirSE, dirS, dirSW, dirW, dirNW}
	}
	f := &JPSPlusFinder{
		Heuristic:        Manhattan,
		DiagonalMovement: opt.DiagonalMovement,
		Weight:           1,
		dirs:             dirs,
		searchSeq:        newSearchSeq(),
	}
	if opt.Heuristic != nil {
		f.Heuristic = opt.Heuristic
	}
	return f
}

// Precompute builds the jump distance table for the given grid.
// Must be called before FindPath. The grid is assumed static.
func (j *JPSPlusFinder) Precompute(grid Grid) {
	j.width = grid.Width()
	j.height = grid.Height()
	size := j.width * j.height * 8
	if cap(j.jumpTable) < size {
		j.jumpTable = make([]int8, size)
	} else {
		j.jumpTable = j.jumpTable[:size]
		for i := range j.jumpTable {
			j.jumpTable[i] = 0
		}
	}

	// Phase 1: detect primary jump points (forced neighbors)
	primaryJPs := j.detectPrimaryJumpPoints(grid)

	// Phase 2: compute cardinal and diagonal distances
	j.computeDistances(grid, primaryJPs)
}

// detectPrimaryJumpPoints scans each walkable cell for forced neighbors.
// Returns a map from cell position to bitmask of directions where the cell
// acts as a jump point.
func (j *JPSPlusFinder) detectPrimaryJumpPoints(grid Grid) map[[2]int]uint8 {
	primaryJPs := make(map[[2]int]uint8)
	w, h := j.width, j.height

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}

			for _, dir := range jpsCardinalDirs {
				nx, ny := x+dir[0], y+dir[1]
				if !grid.IsInside(nx, ny) || grid.IsWalkableAt(nx, ny) {
					continue
				}

				// Perpendicular to wall direction
				var checkDirs [2][2]int
				if dir[0] != 0 { // horizontal direction (E/W)
					checkDirs = [2][2]int{{0, -1}, {0, 1}} // N/S
				} else { // vertical direction (N/S)
					checkDirs = [2][2]int{{-1, 0}, {1, 0}} // W/E
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
					primaryJPs[[2]int{jpx, jpy}] |= 1 << dirIdx
				}
			}
		}
	}
	return primaryJPs
}

// computeDistances fills the jumpTable with distances in all 8 directions.
func (j *JPSPlusFinder) computeDistances(grid Grid, primaryJPs map[[2]int]uint8) {
	w, h := j.width, j.height

	// Helper: get distance to obstacle or jump point in a cardinal direction
	scanCardinal := func(x, y int, dx, dy int, dirIdx int) int8 {
		dist := 0
		for {
			nx, ny := x+dx*(dist+1), y+dy*(dist+1)
			if !grid.IsInside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
				return int8(-dist)
			}
			if primaryJPs[[2]int{nx, ny}]&(1<<dirIdx) != 0 {
				return int8(dist + 1)
			}
			dist++
		}
	}

	// Part A: Cardinal directions
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}
			base := (y*w + x) * 8
			for _, dir := range jpsCardinalDirs {
				dirIdx := jpsCardinalIdx[dir]
				j.jumpTable[base+dirIdx] = scanCardinal(x, y, dir[0], dir[1], dirIdx)
			}
		}
	}

	// Part B: Diagonal directions
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}
			base := (y*w + x) * 8

			for _, ddir := range jpsDiagonalDirs {
				ddirIdx := jpsDiagonalIdx[ddir]
				hdx, hdy := ddir[0], 0 // horizontal component
				vdx, vdy := 0, ddir[1] // vertical component
				hDirIdx := jpsCardinalIdx[[2]int{hdx, hdy}]
				vDirIdx := jpsCardinalIdx[[2]int{vdx, vdy}]

				dist := 0
				for {
					cx, cy := x+ddir[0]*dist, y+ddir[1]*dist
					if !grid.IsInside(cx, cy) || !grid.IsWalkableAt(cx, cy) {
						j.jumpTable[base+ddirIdx] = int8(-dist)
						break
					}

					if dist > 0 {
						// Check cardinal neighbors at this diagonal position
						cb := (cy*w + cx) * 8
						if j.jumpTable[cb+hDirIdx] > 0 || j.jumpTable[cb+vDirIdx] > 0 {
							j.jumpTable[base+ddirIdx] = int8(dist)
							break
						}
					}

					// Check horizontal and vertical neighbors are walkable
					hnx, hny := cx+hdx, cy+hdy
					vnx, vny := cx+vdx, cy+vdy
					if !grid.IsInside(hnx, hny) || !grid.IsWalkableAt(hnx, hny) {
						j.jumpTable[base+ddirIdx] = int8(-dist)
						break
					}
					if !grid.IsInside(vnx, vny) || !grid.IsWalkableAt(vnx, vny) {
						j.jumpTable[base+ddirIdx] = int8(-dist)
						break
					}

					// Next diagonal step
					nx, ny := cx+ddir[0], cy+ddir[1]
					if !grid.IsInside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
						j.jumpTable[base+ddirIdx] = int8(-(dist + 1))
						break
					}
					dist++
				}
			}
		}
	}
}

// getDist returns the precomputed jump distance for cell (x,y) in direction dirIdx.
func (j *JPSPlusFinder) getDist(x, y, dirIdx int) int8 {
	if !j.grid.IsInside(x, y) {
		return 0
	}
	return j.jumpTable[(y*j.width+x)*8+dirIdx]
}

// PrecomputedData serializes the jump table to binary data.
// The returned slice can be saved to a file and loaded later with LoadPrecomputed,
// avoiding the cost of re-running Precompute on server restart.
func (j *JPSPlusFinder) PrecomputedData() ([]byte, error) {
	if j.jumpTable == nil {
		return nil, errors.New("JPSPlusFinder: Precompute must be called before Export")
	}
	// Format: magic(4) + width(4) + height(4) + jumpTable(n)
	size := 12 + len(j.jumpTable)
	buf := make([]byte, size)
	binary.LittleEndian.PutUint32(buf[0:4], 0x4A5053) // "JPS" magic
	binary.LittleEndian.PutUint32(buf[4:8], uint32(j.width))
	binary.LittleEndian.PutUint32(buf[8:12], uint32(j.height))
	for i, v := range j.jumpTable {
		buf[12+i] = byte(v)
	}
	return buf, nil
}

// LoadPrecomputed loads previously exported jump table data.
// Must be called before FindPath with a grid of matching dimensions.
func (j *JPSPlusFinder) LoadPrecomputed(data []byte) error {
	if len(data) < 12 {
		return errors.New("JPSPlusFinder: precomputed data too short")
	}
	magic := binary.LittleEndian.Uint32(data[0:4])
	if magic != 0x4A5053 {
		return fmt.Errorf("JPSPlusFinder: invalid magic 0x%X", magic)
	}
	w := int(binary.LittleEndian.Uint32(data[4:8]))
	h := int(binary.LittleEndian.Uint32(data[8:12]))
	tableSize := w * h * 8
	if len(data[12:]) < tableSize {
		return fmt.Errorf("JPSPlusFinder: precomputed data too short, need %d bytes", tableSize)
	}
	j.width = w
	j.height = h
	if cap(j.jumpTable) < tableSize {
		j.jumpTable = make([]int8, tableSize)
	} else {
		j.jumpTable = j.jumpTable[:tableSize]
	}
	for i := range tableSize {
		j.jumpTable[i] = int8(data[12+i])
	}
	return nil
}

// PrecomputedSize returns the byte size of the serialized jump table.
// Useful for estimating storage or memory before exporting.
func (j *JPSPlusFinder) PrecomputedSize() int {
	if j.jumpTable == nil {
		return 0
	}
	return 12 + len(j.jumpTable)
}

// FindPath performs JPS+ pathfinding using the precomputed jump table.
// Precompute(grid) must have been called first with the same grid.
func (j *JPSPlusFinder) FindPath(startX, startY, endX, endY int, grid Grid) [][2]int {
	if j.jumpTable == nil {
		panic("JPSPlusFinder: Precompute(grid) must be called before FindPath")
	}
	j.searchSeq++
	j.grid = grid
	j.openList = &MinHeap{nodes: j.heapSlice[:0]}
	defer func() { j.heapSlice = j.openList.nodes[:0] }()

	j.startNode = grid.GetNodeAt(startX, startY)
	j.endNode = grid.GetNodeAt(endX, endY)

	if j.startNode == nil || j.endNode == nil {
		return nil
	}
	if !j.startNode.Walkable || !j.endNode.Walkable {
		return nil
	}
	if startX == endX && startY == endY {
		return [][2]int{{startX, startY}}
	}

	j.startNode.ResetSearch(j.searchSeq)
	j.endNode.ResetSearch(j.searchSeq)
	j.startNode.G = 0
	j.startNode.F = 0
	j.openList.Push(j.startNode)

	for !j.openList.Empty() {
		node := j.openList.Pop()
		node.Closed = true

		if node.X == endX && node.Y == endY {
			j.pathBuf = j.pathBuf[:0]
			for n := node; n != nil; n = n.Parent {
				j.pathBuf = append(j.pathBuf, [2]int{n.X, n.Y})
			}
			for i, k := 0, len(j.pathBuf)-1; i < k; i, k = i+1, k-1 {
				j.pathBuf[i], j.pathBuf[k] = j.pathBuf[k], j.pathBuf[i]
			}
			return ExpandPath(j.pathBuf)
		}

		j.identifySuccessors(node)
	}
	return nil
}

func (j *JPSPlusFinder) identifySuccessors(node *Node) {
	x, y := node.X, node.Y
	endX, endY := j.endNode.X, j.endNode.Y
	heuristic := j.Heuristic
	grid := j.grid
	openList := j.openList

	// Use diagonal movement setting to determine which directions to check
	dirs := j.dirs

	for _, dirIdx := range dirs {
		dist := j.getDist(x, y, dirIdx)
		dx, dy := dirToVec(dirIdx)

		// Check if the end node lies in this direction
		if endInDirection(x, y, endX, endY, dx, dy) {
			endDist := absDirDist(endX-x, endY-y, dx, dy)
			obstacleDist := int(-dist) // positive when no jump point (dist < 0)
			if (dist <= 0 && endDist <= obstacleDist) || (dist > 0 && endDist <= int(dist)) {
				// End is reachable in this direction, add it as successor
				gd := octileDist(AbsInt(endX-x), AbsInt(endY-y))
				ng := node.G + gd
				endNode := j.endNode
				endNode.ResetSearch(j.searchSeq)
				if !endNode.Closed && (endNode.Opened == 0 || ng < endNode.G) {
					endNode.G = ng
					if endNode.Opened == 0 {
						endNode.H = 0
					}
					endNode.F = ng
					endNode.Parent = node
					if endNode.Opened == 0 {
						openList.Push(endNode)
						endNode.Opened = 1
					}
				}
				continue
			}
		}

		if dist <= 0 {
			// No jump point in this direction — add the adjacent cell as a
			// regular successor (A* fallback). This lets the search fan out
			// through open space when no forced neighbors exist.
			nx, ny := x+dx, y+dy
			if !grid.IsInside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
				continue
			}
			adjNode := grid.GetNodeAt(nx, ny)
			adjNode.ResetSearch(j.searchSeq)
			if adjNode.Closed {
				continue
			}
			var moveCost float64 = 1
			if dx != 0 && dy != 0 {
				moveCost = SQRT2
			}
			ng := node.G + moveCost
			if adjNode.Opened == 0 || ng < adjNode.G {
				adjNode.G = ng
				if adjNode.Opened == 0 {
					adjNode.H = heuristic(float64(AbsInt(nx-endX)), float64(AbsInt(ny-endY)))
				}
				adjNode.F = adjNode.G + adjNode.H*j.Weight
				adjNode.Parent = node
				if adjNode.Opened == 0 {
					openList.Push(adjNode)
					adjNode.Opened = 1
				} else {
					openList.UpdateItem(adjNode)
				}
			}
			continue
		}

		// Positive dist: jump point exists at this distance
		jx, jy := x+dx*int(dist), y+dy*int(dist)

		if !grid.IsInside(jx, jy) || !grid.IsWalkableAt(jx, jy) {
			continue
		}

		jumpNode := grid.GetNodeAt(jx, jy)
		jumpNode.ResetSearch(j.searchSeq)

		if jumpNode.Closed {
			continue
		}

		gd := octileDist(AbsInt(jx-x), AbsInt(jy-y))
		ng := node.G + gd

		if jumpNode.Opened == 0 || ng < jumpNode.G {
			jumpNode.G = ng
			if jumpNode.Opened == 0 {
				jumpNode.H = heuristic(float64(AbsInt(jx-endX)), float64(AbsInt(jy-endY)))
			}
			jumpNode.F = jumpNode.G + jumpNode.H*j.Weight
			jumpNode.Parent = node

			if jumpNode.Opened == 0 {
				openList.Push(jumpNode)
				jumpNode.Opened = 1
			} else {
				openList.UpdateItem(jumpNode)
			}
		}
	}
}

// endInDirection checks if (ex,ey) is reachable in direction (dx,dy) from (x,y).
func endInDirection(x, y, ex, ey, dx, dy int) bool {
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
func absDirDist(rx, ry, dx, dy int) int {
	if dx != 0 {
		return AbsInt(rx / dx)
	}
	return AbsInt(ry / dy)
}

func (j *JPSPlusFinder) heuristic(x1, y1, x2, y2 int) float64 {
	return j.Heuristic(float64(AbsInt(x2-x1)), float64(AbsInt(y2-y1)))
}

func octileDist(dx, dy int) float64 {
	f := SQRT2 - 1
	if dx < dy {
		return f*float64(dx) + float64(dy)
	}
	return f*float64(dy) + float64(dx)
}

func dirToVec(dirIdx int) (int, int) {
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
