package grid

import (
	"fmt"
	"strings"

	"github.com/actfuns/pathfinding/finder"
)

// GridType identifies the tile grid layout.
type GridType int

const (
	// Orthogonal is a standard square-tile grid (4-directional or 8-directional).
	Orthogonal GridType = iota
	// Staggered is a 45-degree isometric/staggered grid (diamond-shaped tiles).
	Staggered
	// Hexagonal is a hexagonal grid (both pointy-top and flat-top layouts).
	Hexagonal
)

// NewMatrix creates an empty (all zero) matrix with the given dimensions.
// All cells are 0 (walkable), ready to pass to NewOrthogonalGrid, NewHexGrid,
// or NewStaggeredGrid.
func NewMatrix(w, h int) [][]int {
	m := make([][]int, h)
	for i := range m {
		m[i] = make([]int, w)
	}
	return m
}

// Grid defines the interface that all grid types implement.
// It focuses on grid-consumer operations: coordinate conversion, walkability
// queries, pathfinding, and rendering. This is separate from finder.Grid,
// which only exposes what pathfinding algorithms need.
// SVGOpts configures SVG rendering behavior. nil means use defaults.
type SVGOpts struct {
	// WeightColors maps weight thresholds to tile fill colors.
	// Each entry is (threshold, color). Applied in order: weight <= threshold → color.
	// The last entry with a threshold of +Inf catches everything above the second-last.
	// The second-to-last entry with a threshold of +Inf catches exactly the last threshold.
	// Example:
	//   {{"0.3", "#bbdefb"}, {"1.0", "#c8e6c9"}, {"2.0", "#fff9c4"}, {"5.0", "#ffcc80"}, {"+Inf", "#ef5350"}}
	// Set to nil for defaults: road(0.3)→blue, grass(1.0)→green, sand(2.0)→yellow,
	// swamp(5.0)→orange, >5.0→red.
	WeightColors [][2]string
	// LegendEntries overrides the legend labels. Each entry is (color_hex, label).
	// If nil, labels are auto-generated from WeightColors as "≤threshold".
	LegendEntries [][2]string
}

// DefaultSVGOpts is the default SVG rendering config, used when nil is passed.
var DefaultSVGOpts = &SVGOpts{
	WeightColors: [][2]string{
		{"0.3", "#bbdefb"},
		{"1.0", "#c8e6c9"},
		{"2.0", "#fff9c4"},
		{"5.0", "#ffcc80"},
		{"+Inf", "#ef5350"},
	},
	LegendEntries: [][2]string{
		{"#bbdefb", "Road ≤0.3"},
		{"#c8e6c9", "Grass ≤1.0"},
		{"#fff9c4", "Sand ≤2.0"},
		{"#ffcc80", "Swamp ≤5.0"},
		{"#ef5350", ">5.0 Extreme"},
		{"#555555", "Wall"},
	},
}

// Grid defines the interface that all grid types implement.
// It focuses on grid-consumer operations: coordinate conversion, walkability
// queries, pathfinding, and rendering. This is separate from finder.Grid,
// which only exposes what pathfinding algorithms need.
type Grid interface {
	finder.Grid

	// SetWalkableAt sets the walkability of the tile (x, y).
	SetWalkableAt(x, y int, walkable bool)
	// ObstacleCount returns the number of non-walkable tiles.
	ObstacleCount() int
	// TileIndex returns the flat array index for tile (x, y).
	// Equivalent to y*Width + x. Panics if outside the grid.
	TileIndex(x, y int) int
	// TileXY returns the tile coordinates for a flat array index.
	// Equivalent to (index % Width, index / Width).
	TileXY(index int) (int, int)

	// SetWeightAt sets the per-tile movement cost multiplier for tile (x, y).
	// Weight 1.0 is the default; higher values make movement more expensive.
	SetWeightAt(x, y int, weight float64)
	// GetWeightAt returns the movement cost multiplier for tile (x, y).
	// Returns 1.0 for tiles outside the grid.
	GetWeightAt(x, y int) float64

	// WorldToTile converts a world-space coordinate to tile-space.
	// Results outside the grid should be checked with IsInside.
	WorldToTile(wx, wy float32) (int, int)
	// TileToWorld converts a tile coordinate to world-space, returning
	// the centre point of the tile.
	TileToWorld(tx, ty int) (float32, float32)
	// IsWalkableAtWorld reports whether the tile at world position (wx, wy) is walkable.
	IsWalkableAtWorld(wx, wy float32) bool
	// SetWalkableAtWorld sets the walkability of the tile at world position (wx, wy).
	SetWalkableAtWorld(wx, wy float32, walkable bool)

	// FindNearestWalkable is like FindNearestWalkable but returns tile
	// coordinates instead of edge-clamped world coordinates.
	FindNearestWalkable(wx, wy float32, maxRadius int) (int, int, bool)
	// FindNearestWalkableWorld finds the nearest walkable tile within maxRadius
	// (Chebyshev distance in tiles) from world position (wx, wy).
	// Returns the closest point on the edge of the nearest walkable tile, or
	// (0, 0, false) if no walkable tile exists within the search radius.
	FindNearestWalkableWorld(wx, wy float32, maxRadius int, edgeInset float32) (float32, float32, bool)

	// Finder returns the pathfinder used by this grid.
	Finder() finder.Finder
	// FindPath finds a path between two tile positions.
	// Result is backed by an internal buffer — valid only until the next FindPath call.
	FindPath(x1, y1, x2, y2 int) [][2]int
	// FindPathWorld finds a path between two world positions through the grid.
	// Uses the grid's Finder. Returns the path in world coordinates.
	// Result is backed by an internal buffer — valid only until the next FindPathWorld call.
	FindPathWorld(wx1, wy1, wx2, wy2 float32) [][2]float32
	// FindSmoothPath finds a tile path and smooths it via LOS string-pulling.
	// Same buffer contract as FindPath.
	FindSmoothPath(x1, y1, x2, y2 int) [][2]int
	// FindSmoothPathWorld finds a path and smooths it via LOS string-pulling.
	// Same buffer contract as FindPath.
	FindSmoothPathWorld(wx1, wy1, wx2, wy2 float32) [][2]float32
	// SmoothenPath removes unnecessary waypoints from a tile path
	// by checking line-of-sight between each point.
	SmoothenPath(path [][2]int) [][2]int

	// RenderSVG renders the grid and paths as an SVG string.
	// Tiles are coloured by their Weight value (see DefaultSVGOpts).
	// Multiple paths are drawn in different colours (1st=blue, 2nd=red dashed).
	// Start/end markers are derived from the first path.
	// Pass nil to use DefaultSVGOpts.
	RenderSVG(cfg *SVGOpts, paths ...[][2]int) string
}

func drawPathAndMarkers(b *strings.Builder, startX, startY, endX, endY int,
	centerOf func(tx, ty int) (float64, float64), paths ...[][2]int) {

	pathColors := []string{"#0066cc", "#e53935", "#2e7d32", "#7b1fa2"}
	for pi, path := range paths {
		color := pathColors[pi%len(pathColors)]
		dash := ""
		if pi == 1 {
			dash = ` stroke-dasharray="6,4"`
		}
		if len(path) > 0 {
			pts := make([]string, len(path))
			for i, p := range path {
				cx, cy := centerOf(p[0], p[1])
				pts[i] = fmt.Sprintf("%.1f,%.1f", cx, cy)
			}
			fmt.Fprintf(b, `<polyline points="%s" fill="none" stroke="%s" stroke-width="3"%s stroke-linejoin="round" stroke-linecap="round"/>`+"\n",
				strings.Join(pts, " "), color, dash)
		}
	}

	sx, sy := centerOf(startX, startY)
	fmt.Fprintf(b, `<circle cx="%.1f" cy="%.1f" r="6" fill="#00cc44" stroke="#009933" stroke-width="2"/>`+"\n", sx, sy)

	ex, ey := centerOf(endX, endY)
	fmt.Fprintf(b, `<circle cx="%.1f" cy="%.1f" r="6" fill="#cc0000" stroke="#990000" stroke-width="2"/>`+"\n", ex, ey)
}

// weightToColor maps a tile weight to a fill color using cfg.WeightColors.
// Falls back to the hardcoded defaults when cfg or cfg.WeightColors is nil.
func weightToColor(w float64, cfg *SVGOpts) string {
	for _, entry := range cfg.WeightColors {
		threshold := float64(1 << 62)
		if entry[0] != "+Inf" {
			fmt.Sscanf(entry[0], "%f", &threshold)
		}
		if w <= threshold {
			return entry[1]
		}
	}
	return cfg.WeightColors[len(cfg.WeightColors)-1][1]
}
func renderLegend(b *strings.Builder, legX, legY float64, cfg *SVGOpts) {
	// Build legend entries from config or auto-generate from WeightColors
	var entries [][2]string
	if cfg.LegendEntries != nil {
		entries = cfg.LegendEntries
	} else {
		for _, wc := range cfg.WeightColors {
			label := ">" + cfg.WeightColors[0][0]
			if wc[0] != "+Inf" {
				label = "≤" + wc[0]
			}
			entries = append(entries, [2]string{wc[1], label})
		}
		// Always append Wall entry
		entries = append(entries, [2]string{"#555555", "Wall"})
	}
	legH := float64(20 + len(entries)*26 + 10)
	// Legend background
	fmt.Fprintf(b, `<rect x="%.1f" y="%.1f" width="110" height="%.1f" fill="white" stroke="#bbb" stroke-width="1" rx="4"/>`+"\n",
		legX, legY, legH)
	fmt.Fprintf(b, `<text x="%.1f" y="%.1f" font-size="12" font-family="sans-serif" font-weight="bold" fill="#333">Terrain</text>`+"\n",
		legX+8, legY+16)

	for i, e := range entries {
		ey := legY + 28 + float64(i*26)
		fmt.Fprintf(b, `<rect x="%.1f" y="%.1f" width="14" height="14" fill="%s" stroke="#999" stroke-width="1" rx="2"/>`+"\n",
			legX+8, ey, e[0])
		fmt.Fprintf(b, `<text x="%.1f" y="%.1f" font-size="11" font-family="sans-serif" fill="#333">%s</text>`+"\n",
			legX+28, ey+12, e[1])
	}
}

// xmlHeader returns the SVG XML declaration and root element.
func xmlHeader(w, h int) string {
	return fmt.Sprintf(`<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">
`, w, h, w, h)
}
