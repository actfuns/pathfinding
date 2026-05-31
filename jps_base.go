package pathfinding

// JumpPointFinderBase is the base implementation for Jump Point Search.
// Uses function pointers (jumpFn, findNeighborsFn) for strategy dispatch
// since Go does not have virtual methods.
type JumpPointFinderBase struct {
	Heuristic        HeuristicFunc
	TrackRecursion   bool
	grid             *Grid
	startNode        *Node
	endNode          *Node
	openList         *MinHeap

	jumpFn          func(b *JumpPointFinderBase, x, y, px, py int) *[2]int
	findNeighborsFn func(b *JumpPointFinderBase, node *Node) [][2]int
}

// NewJumpPointFinderBase creates a new JumpPointFinderBase.
func NewJumpPointFinderBase(opt *FinderOptions) *JumpPointFinderBase {
	f := &JumpPointFinderBase{
		Heuristic:      Manhattan,
		TrackRecursion: false,
	}
	if opt != nil {
		if opt.Heuristic != nil {
			f.Heuristic = opt.Heuristic
		}
		f.TrackRecursion = opt.TrackRecursion
	}
	return f
}

// FindPath finds a path using JPS.
func (b *JumpPointFinderBase) FindPath(startX, startY, endX, endY int, grid *Grid) [][2]int {
	b.openList = NewMinHeap(func(a, b *Node) bool {
		return a.f < b.f
	})
	b.startNode = grid.GetNodeAt(startX, startY)
	b.endNode = grid.GetNodeAt(endX, endY)
	b.grid = grid

	b.startNode.g = 0
	b.startNode.f = 0
	b.openList.Push(b.startNode)
	b.startNode.opened = 1

	for !b.openList.Empty() {
		node := b.openList.Pop()
		node.closed = true

		if node == b.endNode {
			return ExpandPath(Backtrace(b.endNode))
		}

		b.identifySuccessors(node)
	}

	return nil
}

func (b *JumpPointFinderBase) identifySuccessors(node *Node) {
	grid := b.grid
	heuristic := b.Heuristic
	openList := b.openList
	endX := b.endNode.X
	endY := b.endNode.Y
	x, y := node.X, node.Y

	neighbors := b.findNeighborsFn(b, node)
	for _, neighbor := range neighbors {
		jumpPoint := b.jumpFn(b, neighbor[0], neighbor[1], x, y)
		if jumpPoint == nil {
			continue
		}

		jx, jy := jumpPoint[0], jumpPoint[1]
		jumpNode := grid.GetNodeAt(jx, jy)

		if jumpNode.closed {
			continue
		}

		d := Octile(float64(absInt(jx-x)), float64(absInt(jy-y)))
		ng := node.g + d

		if jumpNode.opened == 0 || ng < jumpNode.g {
			jumpNode.g = ng
			if jumpNode.opened == 0 {
				jumpNode.h = heuristic(float64(absInt(jx-endX)), float64(absInt(jy-endY)))
			}
			jumpNode.f = jumpNode.g + jumpNode.h
			jumpNode.parent = node

			if jumpNode.opened == 0 {
				openList.Push(jumpNode)
				jumpNode.opened = 1
			} else {
				openList.UpdateItem(jumpNode)
			}
		}
	}
}
