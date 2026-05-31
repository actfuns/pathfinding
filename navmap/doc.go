// Package navmap provides tile map types with coordinate conversion and pathfinding.
//
// A TileMap wraps a core.Grid with a CoordConverter, allowing pathfinding
// in world coordinates rather than tile coordinates. Supports Orthogonal,
// Hexagonal (pointy/flat), and Staggered (isometric) map layouts.
//
// Reference:
//   - pmcxs-hexgrid: Hex cube coordinates, HexToPixel/PixelToHex math
//   - gonutz-tiled: TMX map file reader
//   - hugoscurti-hierarchical-pathfinding: Map.cs for tile-based map loading
package navmap
