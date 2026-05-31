package navmap

import (
	"github.com/parasol/pathfinding/core"
	"github.com/parasol/pathfinding/finder"
	"github.com/parasol/pathfinding/hpa"
)

// TileMap wraps a core.Grid with coordinate conversion and pathfinding.
// Users interact with world coordinates rather than tile coordinates.
type TileMap struct {
	Grid      *core.Grid
	Converter CoordConverter
	finder    core.Finder
	hpaworld  *hpa.HPAWorld // nil when HPA* not in use
}

// FindPath finds a path between two world positions through the tile map.
// Returns world-coordinate waypoints.
func (m *TileMap) FindPath(worldStartX, worldStartY, worldEndX, worldEndY float64) [][2]float64 {
	sx, sy := m.Converter.WorldToTile(worldStartX, worldStartY)
	ex, ey := m.Converter.WorldToTile(worldEndX, worldEndY)

	// Bounds check
	if !m.Grid.IsInside(sx, sy) || !m.Grid.IsInside(ex, ey) {
		return nil
	}

	if m.hpaworld != nil {
		hfinder := hpa.NewHPAStarFinder(nil)
		result := hfinder.FindPath(sx, sy, ex, ey, m.Grid, m.hpaworld)
		if result.Waypoints == nil {
			return nil
		}
		return m.convertPath(result.Waypoints)
	}

	path := m.finder.FindPath(sx, sy, ex, ey, m.Grid)
	if path == nil {
		return nil
	}
	return m.convertPath(path)
}

func (m *TileMap) convertPath(tilePath [][2]int) [][2]float64 {
	wp := make([][2]float64, len(tilePath))
	for i, p := range tilePath {
		wx, wy := m.Converter.TileToWorld(p[0], p[1])
		wp[i] = [2]float64{wx, wy}
	}
	return wp
}

// IsWalkableAt returns whether the tile at the given world position is walkable.
func (m *TileMap) IsWalkableAt(wx, wy float64) bool {
	tx, ty := m.Converter.WorldToTile(wx, wy)
	if tx < 0 || ty < 0 {
		return false
	}
	return m.Grid.IsWalkableAt(tx, ty)
}

// SetWalkableAt sets the walkable status of the tile at the given world position.
func (m *TileMap) SetWalkableAt(wx, wy float64, walkable bool) {
	tx, ty := m.Converter.WorldToTile(wx, wy)
	if tx >= 0 && ty >= 0 {
		m.Grid.SetWalkableAt(tx, ty, walkable)
	}
}

// --- Builder ---

// TileMapBuilder builds a TileMap with specified options.
type TileMapBuilder struct {
	converter CoordConverter
	grid      *core.Grid
	finder    core.Finder
	enableHPA bool
	hpaChunk  int
}

// NewTileMapBuilder creates a new TileMapBuilder.
func NewTileMapBuilder() *TileMapBuilder {
	return &TileMapBuilder{
		hpaChunk: 16,
	}
}

// WithGrid sets the underlying grid from a matrix (0=walkable, non-zero=obstacle).
func (b *TileMapBuilder) WithGrid(matrix [][]int) *TileMapBuilder {
	b.grid = core.NewGrid(matrix)
	return b
}

// WithCoreGrid sets the grid directly from a core.Grid.
func (b *TileMapBuilder) WithCoreGrid(g *core.Grid) *TileMapBuilder {
	b.grid = g
	return b
}

// WithOrthogonalMap sets the map to orthogonal with given tile dimensions.
func (b *TileMapBuilder) WithOrthogonalMap(tileWidth, tileHeight float64) *TileMapBuilder {
	b.converter = NewOrthogonalConverter(tileWidth, tileHeight)
	return b
}

// WithHexMap sets the map to hexagonal with given hex size.
func (b *TileMapBuilder) WithHexMap(size float64, pointy bool) *TileMapBuilder {
	if pointy {
		b.converter = NewPointyHexConverter(size)
	} else {
		b.converter = NewFlatHexConverter(size)
	}
	return b
}

// WithConverter sets a custom CoordConverter.
func (b *TileMapBuilder) WithConverter(c CoordConverter) *TileMapBuilder {
	b.converter = c
	return b
}

// WithAStar configures A* as the pathfinder.
func (b *TileMapBuilder) WithAStar(opt *finder.FinderOptions) *TileMapBuilder {
	b.finder = finder.NewAStarFinder(opt)
	return b
}

// WithJPF configures JPS as the pathfinder.
func (b *TileMapBuilder) WithJPF(opt *finder.FinderOptions) *TileMapBuilder {
	b.finder = finder.JumpPointFinder(opt)
	return b
}

// WithBFS configures BFS as the pathfinder.
func (b *TileMapBuilder) WithBFS(opt *finder.FinderOptions) *TileMapBuilder {
	b.finder = finder.NewBreadthFirstFinder(opt)
	return b
}

// WithDijkstra configures Dijkstra as the pathfinder.
func (b *TileMapBuilder) WithDijkstra(opt *finder.FinderOptions) *TileMapBuilder {
	b.finder = finder.NewDijkstraFinder(opt)
	return b
}

// WithFinder sets a custom finder.
func (b *TileMapBuilder) WithFinder(f core.Finder) *TileMapBuilder {
	b.finder = f
	return b
}

// WithHPA enables HPA* acceleration with the given chunk size.
func (b *TileMapBuilder) WithHPA(chunkSize int) *TileMapBuilder {
	b.enableHPA = true
	b.hpaChunk = chunkSize
	return b
}

// Build constructs the TileMap. Must call at least one of WithGrid/WithCoreGrid
// and one of With*Map/WithConverter before Build.
func (b *TileMapBuilder) Build() *TileMap {
	if b.converter == nil {
		b.converter = NewOrthogonalConverter(1, 1)
	}

	tm := &TileMap{
		Grid:      b.grid,
		Converter: b.converter,
	}

	if b.finder == nil {
		b.finder = finder.NewAStarFinder(nil)
	}
	tm.finder = b.finder

	if b.enableHPA && b.grid != nil {
		hpab := hpa.NewHPABuilder(hpa.HPAConfig{ChunkSize: b.hpaChunk})
		tm.hpaworld = hpab.Build(b.grid)
		// HPA uses its own finder strategy
		// When hpaworld is set, FindPath uses HPAStarFinder directly
	}

	return tm
}
