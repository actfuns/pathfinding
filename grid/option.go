package grid

import (
	"github.com/actfuns/pathfinding/finder"
)

// SmoothMode specifies the path smoothing algorithm.
type SmoothMode int

const (
	SmoothNone            SmoothMode = iota // no smoothing
	SmoothBresenham                         // Bresenham line-of-sight (orthogonal)
	SmoothBresenhamStrict                   // Bresenham with corner-cutting prevention (orthogonal)
	SmoothDense                             // world-space dense sampling (hex, staggered)
)

// gridOptions aggregates all configurable settings for grid construction.
type gridOptions struct {
	tileW, tileH int
	finder       finder.Finder
	smoothMode   SmoothMode
	hexSide      int
	staggerX     bool
	staggerEven  bool
	staggerAxis  string
	staggerIndex string
}

// GridOption configures any grid type. Apply via constructor opts (NewOrthogonalGrid, etc.).
type GridOption func(*gridOptions)

// --- Shared options (apply to all grid types) ---

// WithTileSize sets the tile dimensions for world coordinate conversion.
func WithTileSize(w, h int) GridOption {
	return func(o *gridOptions) { o.tileW = w; o.tileH = h }
}

// WithFinder sets the pathfinding algorithm (e.g., finder.NewAStar(), finder.NewBFS()).
func WithFinder(f finder.Finder) GridOption {
	return func(o *gridOptions) { o.finder = f }
}

// WithSmoothBresenham enables Bresenham line-of-sight path smoothing (orthogonal grids only).
func WithSmoothBresenham() GridOption {
	return func(o *gridOptions) { o.smoothMode = SmoothBresenham }
}

// WithSmoothBresenhamStrict enables Bresenham smoothing with corner-cutting prevention (orthogonal grids only).
func WithSmoothBresenhamStrict() GridOption {
	return func(o *gridOptions) { o.smoothMode = SmoothBresenhamStrict }
}

// WithSmoothDense enables world-space dense-sampling path smoothing (hex and staggered grids only).
func WithSmoothDense() GridOption {
	return func(o *gridOptions) { o.smoothMode = SmoothDense }
}

// --- Hex grid options ---

// WithHexSide sets the side length (in pixels) of a hex tile for world coordinate conversion.
func WithHexSide(side int) GridOption {
	return func(o *gridOptions) { o.hexSide = side }
}

// WithHexFlatTop configures the hex grid for flat-top (pointy-side) orientation.
func WithHexFlatTop() GridOption {
	return func(o *gridOptions) { o.staggerX = true }
}

// WithHexEvenStagger configures the hex grid to use even-row staggering.
func WithHexEvenStagger() GridOption {
	return func(o *gridOptions) { o.staggerEven = true }
}

// --- Staggered grid options ---

// WithStaggerAxis sets the stagger axis for isometric staggered grids ("x" or "y").
func WithStaggerAxis(axis string) GridOption {
	return func(o *gridOptions) { o.staggerAxis = axis }
}

// WithStaggerEvenIndex configures the staggered grid to use even-row indexing.
func WithStaggerEvenIndex() GridOption {
	return func(o *gridOptions) { o.staggerIndex = "even" }
}
