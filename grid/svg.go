package grid

import (
	"fmt"
	"math"
	"strings"
)

// RenderSVG renders the grid and an optional path as an SVG string.
// If path is nil, only the grid is rendered.
func (g *OrthogonalGrid) RenderSVG(path [][2]int, startX, startY, endX, endY int) string {
	cellW := 40
	cellH := 40
	padding := 20
	width := g.width*cellW + padding*2
	height := g.height*cellH + padding*2

	var b strings.Builder
	b.WriteString(xmlHeader(width, height))

	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			rx := padding + x*cellW
			ry := padding + y*cellH
			fill := "#ffffff"
			stroke := "#cccccc"
			if !g.IsWalkableAt(x, y) {
				fill = "#333333"
				stroke = "#333333"
			}
			fmt.Fprintf(&b, `<rect x="%d" y="%d" width="%d" height="%d" fill="%s" stroke="%s" stroke-width="1"/>`+"\n",
				rx, ry, cellW, cellH, fill, stroke)
		}
	}

	drawPathAndMarkers(&b, path, startX, startY, endX, endY,
		func(tx, ty int) (float64, float64) {
			return float64(padding + tx*cellW + cellW/2),
				float64(padding + ty*cellH + cellH/2)
		})

	b.WriteString("</svg>\n")
	return b.String()
}

// RenderSVG renders the hex grid and an optional path as an SVG string.
func (g *HexGrid) RenderSVG(path [][2]int, startX, startY, endX, endY int) string {
	padding := 20.0

	// Find bounds of all tile center points
	minX, minY := math.MaxFloat64, math.MaxFloat64
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, y)
			if cx < minX {
				minX = cx
			}
			if cy < minY {
				minY = cy
			}
		}
	}

	// Build hex polygon vertices relative to tile center
	var hexOffsets [][2]float64
	if g.staggerX {
		// Flat-top: pointy top and bottom, flat top and bottom
		h := float64(g.renH)
		w := float64(g.tileW)
		side := float64(g.hexSide) / 2
		hexOffsets = [][2]float64{
			{side, 0},
			{w - side, 0},
			{w, h / 2},
			{w - side, h},
			{side, h},
			{0, h / 2},
		}
	} else {
		// Pointy-top: flat top and bottom, pointy left and right
		w := float64(g.renW)
		h := float64(g.tileH)
		side := float64(g.hexSide) / 2
		hexOffsets = [][2]float64{
			{w / 2, 0},
			{w, side},
			{w, h - side},
			{w / 2, h},
			{0, h - side},
			{0, side},
		}
	}

	// Find bounds of all hex vertices to compute SVG size
	vxMin, vyMin := math.MaxFloat64, math.MaxFloat64
	vxMax, vyMax := -math.MaxFloat64, -math.MaxFloat64
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, y)
			for _, off := range hexOffsets {
				vx := cx + off[0]
				vy := cy + off[1]
				if vx < vxMin {
					vxMin = vx
				}
				if vy < vyMin {
					vyMin = vy
				}
				if vx > vxMax {
					vxMax = vx
				}
				if vy > vyMax {
					vyMax = vy
				}
			}
		}
	}

	svgW := (vxMax - vxMin) + padding*2
	svgH := (vyMax - vyMin) + padding*2
	dx := padding - vxMin
	dy := padding - vyMin

	var b strings.Builder
	b.WriteString(xmlHeader(int(math.Ceil(svgW)), int(math.Ceil(svgH))))

	// Draw tiles
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, y)
			fill := "#ffffff"
			stroke := "#cccccc"
			if !g.IsWalkableAt(x, y) {
				fill = "#333333"
				stroke = "#333333"
			}
			pts := make([]string, len(hexOffsets))
			for i, off := range hexOffsets {
				pts[i] = fmt.Sprintf("%.1f,%.1f", cx+off[0]+dx, cy+off[1]+dy)
			}
			fmt.Fprintf(&b, `<polygon points="%s" fill="%s" stroke="%s" stroke-width="1"/>`+"\n",
				strings.Join(pts, " "), fill, stroke)
		}
	}

	drawPathAndMarkers(&b, path, startX, startY, endX, endY,
		func(tx, ty int) (float64, float64) {
			cx, cy := g.tileToScreenCoords(tx, ty)
			return cx + dx, cy + dy
		})

	b.WriteString("</svg>\n")
	return b.String()
}

// RenderSVG renders the staggered grid and an optional path as an SVG string.
func (g *StaggeredGrid) RenderSVG(path [][2]int, startX, startY, endX, endY int) string {
	padding := 20.0
	tileW := float64(g.tileW)
	tileH := float64(g.tileH)

	// Diamond vertices relative to tile center
	diamond := [][2]float64{
		{tileW / 2, 0},
		{tileW, tileH / 2},
		{tileW / 2, tileH},
		{0, tileH / 2},
	}

	// Find bounds of all diamond vertices
	vxMin, vyMin := math.MaxFloat64, math.MaxFloat64
	vxMax, vyMax := -math.MaxFloat64, -math.MaxFloat64
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, y)
			for _, off := range diamond {
				vx := cx + off[0]
				vy := cy + off[1]
				if vx < vxMin {
					vxMin = vx
				}
				if vy < vyMin {
					vyMin = vy
				}
				if vx > vxMax {
					vxMax = vx
				}
				if vy > vyMax {
					vyMax = vy
				}
			}
		}
	}

	svgW := (vxMax - vxMin) + padding*2
	svgH := (vyMax - vyMin) + padding*2
	dx := padding - vxMin
	dy := padding - vyMin

	var b strings.Builder
	b.WriteString(xmlHeader(int(math.Ceil(svgW)), int(math.Ceil(svgH))))

	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, y)
			fill := "#ffffff"
			stroke := "#cccccc"
			if !g.IsWalkableAt(x, y) {
				fill = "#333333"
				stroke = "#333333"
			}
			pts := make([]string, len(diamond))
			for i, off := range diamond {
				pts[i] = fmt.Sprintf("%.1f,%.1f", cx+off[0]+dx, cy+off[1]+dy)
			}
			fmt.Fprintf(&b, `<polygon points="%s" fill="%s" stroke="%s" stroke-width="1"/>`+"\n",
				strings.Join(pts, " "), fill, stroke)
		}
	}

	drawPathAndMarkers(&b, path, startX, startY, endX, endY,
		func(tx, ty int) (float64, float64) {
			cx, cy := g.tileToScreenCoords(tx, ty)
			return cx + dx, cy + dy
		})

	b.WriteString("</svg>\n")
	return b.String()
}

// drawPathAndMarkers draws the path polyline and start/end markers.
// centerOf returns the SVG pixel coordinates for a tile.
func drawPathAndMarkers(b *strings.Builder, path [][2]int, startX, startY, endX, endY int,
	centerOf func(tx, ty int) (float64, float64)) {

	if len(path) > 0 {
		pts := make([]string, len(path))
		for i, p := range path {
			cx, cy := centerOf(p[0], p[1])
			pts[i] = fmt.Sprintf("%.1f,%.1f", cx, cy)
		}
		fmt.Fprintf(b, `<polyline points="%s" fill="none" stroke="#0066cc" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>`+"\n",
			strings.Join(pts, " "))
	}

	sx, sy := centerOf(startX, startY)
	fmt.Fprintf(b, `<circle cx="%.1f" cy="%.1f" r="6" fill="#00cc44" stroke="#009933" stroke-width="2"/>`+"\n", sx, sy)

	ex, ey := centerOf(endX, endY)
	fmt.Fprintf(b, `<circle cx="%.1f" cy="%.1f" r="6" fill="#cc0000" stroke="#990000" stroke-width="2"/>`+"\n", ex, ey)
}

func xmlHeader(w, h int) string {
	return fmt.Sprintf(`<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">
`, w, h, w, h)
}
