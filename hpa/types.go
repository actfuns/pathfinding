package hpa

// HPAConfig holds configuration constants for HPA*.
type HPAConfig struct {
	ChunkSize int
}

// DefaultHPAConfig returns a default HPA* configuration with 16×16 chunks.
func DefaultHPAConfig() HPAConfig {
	return HPAConfig{ChunkSize: 16}
}

// Portal represents a portal on a chunk border or a diagonal corner portal.
type Portal struct {
	CenterX int // world X of portal center
	CenterY int // world Y of portal center
	Length  int // number of cells this portal spans
	Offset  int // offset along the chunk edge (0-based)

	// External connections to neighboring chunks (portal keys)
	ExternalCount   int
	ExternalPortals [5]int // diagonal can need up to 5

	// Internal connections to other portals in the same chunk
	InternalCount   int
	InternalPortals [255]int // portal key of connected portal
	InternalCosts   [255]int // movement cost (octile * 10)
}

func newPortal(cx, cy int) *Portal {
	return &Portal{
		CenterX: cx,
		CenterY: cy,
	}
}

// PortalKey encodes chunk ID, edge position, and direction.
// key = position + direction*ChunkSize + chunkID*MaxPortalsPerChunk
func PortalKey(chunkID, pos, dir, chunkSize int) int {
	return pos + dir*chunkSize + chunkID*MaxPortalsPerChunk
}

func portalKeyChunkID(key, chunkSize int) int {
	return key / MaxPortalsPerChunk
}

func portalKeyPos(key, chunkSize int) int {
	return key % (chunkSize * 4) % chunkSize
}

func portalKeyDir(key, chunkSize int) int {
	return key % (chunkSize * 4) / chunkSize
}

// MaxPortalsPerChunk = ChunkSize * 4 (N/E/S/W) + possible diagonal portals
const MaxPortalsPerChunk = 256

// HPAWorld holds the hierarchical portal graph for a Grid.
type HPAWorld struct {
	Portals      []*Portal // index = portal key
	NumPortals   int       // how many portal slots are actually used
	ChunkMapX    int
	ChunkMapY    int
	PaddedWidth  int
	PaddedHeight int
	ChunkSize    int
}
