package finder

import (
	"github.com/parasol/pathfinding/core"
)

// JumpPointFinder creates a JPS finder based on the diagonal movement setting.
func JumpPointFinder(opt *FinderOptions) core.Finder {
	if opt == nil {
		opt = &FinderOptions{}
	}
	switch opt.DiagonalMovement {
	case core.DiagonalNever:
		return NewJPFNeverMoveDiagonally(opt)
	case core.DiagonalAlways:
		return NewJPFAlwaysMoveDiagonally(opt)
	case core.DiagonalOnlyWhenNoObstacles:
		return NewJPFMoveDiagonallyIfNoObstacles(opt)
	default:
		return NewJPFMoveDiagonallyIfAtMostOneObstacle(opt)
	}
}

// JumpPointFinderBase is the base implementation for Jump Point Search.
type JumpPointFinderBase struct {
	Heuristic        core.HeuristicFunc
	TrackRecursion   bool
	grid             *core.Grid
	startNode        *core.Node
	endNode          *core.Node
	openList         *core.MinHeap

	jumpFn          func(b *JumpPointFinderBase, x, y, px, py int) *[2]int
	findNeighborsFn func(b *JumpPointFinderBase, node *core.Node) [][2]int
}

// NewJumpPointFinderBase creates a new JumpPointFinderBase.
func NewJumpPointFinderBase(opt *FinderOptions) *JumpPointFinderBase {
	f := &JumpPointFinderBase{
		Heuristic:      core.Manhattan,
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
func (b *JumpPointFinderBase) FindPath(startX, startY, endX, endY int, grid *core.Grid) [][2]int {
	b.openList = core.NewMinHeap(func(a, bNode *core.Node) bool {
		return a.F < bNode.F
	})
	b.startNode = grid.GetNodeAt(startX, startY)
	b.endNode = grid.GetNodeAt(endX, endY)
	b.grid = grid

	b.startNode.G = 0
	b.startNode.F = 0
	b.openList.Push(b.startNode)
	b.startNode.Opened = 1

	for !b.openList.Empty() {
		node := b.openList.Pop()
		node.Closed = true

		if node == b.endNode {
			return core.ExpandPath(core.Backtrace(b.endNode))
		}

		b.identifySuccessors(node)
	}

	return nil
}

func (b *JumpPointFinderBase) identifySuccessors(node *core.Node) {
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

		if jumpNode.Closed {
			continue
		}

		d := core.Octile(float64(core.AbsInt(jx-x)), float64(core.AbsInt(jy-y)))
		ng := node.G + d

		if jumpNode.Opened == 0 || ng < jumpNode.G {
			jumpNode.G = ng
			if jumpNode.Opened == 0 {
				jumpNode.H = heuristic(float64(core.AbsInt(jx-endX)), float64(core.AbsInt(jy-endY)))
			}
			jumpNode.F = jumpNode.G + jumpNode.H
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
