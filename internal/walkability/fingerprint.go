package walkability

// Toggle updates an obstacle-layout fingerprint when a tile changes walkability.
// Each tile contributes a mixed value derived from its row-major index.
func Toggle(fingerprint, index uint64) uint64 {
	mixed := index + 0x9e3779b97f4a7c15
	mixed = (mixed ^ (mixed >> 30)) * 0xbf58476d1ce4e5b9
	mixed = (mixed ^ (mixed >> 27)) * 0x94d049bb133111eb
	return fingerprint ^ (mixed ^ (mixed >> 31))
}
