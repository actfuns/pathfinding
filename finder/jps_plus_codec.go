package finder

import (
	"encoding/binary"
	"errors"
	"fmt"
	"hash/crc32"
)

// Jump-table format magics: JPS (int8) and JPS2 (int16).
const (
	jpsPlusMagic8          uint32 = 0x4A5053
	jpsPlusMagic16         uint32 = 0x4A505332
	jpsPlusCanonicalFlag   uint32 = 1 << 31
	jpsPlusFingerprintFlag uint32 = 1 << 29
	jpsPlusChecksumFlag    uint32 = 1 << 28
)

// PrecomputedData serializes the jump table to binary data.
// The returned slice can be saved to a file and loaded later with LoadPrecomputed,
// avoiding the cost of re-running Precompute on server restart.
func (j *JPSPlusFinder) PrecomputedData() ([]byte, error) {
	if !j.hasTable() {
		return nil, errors.New("JPSPlusFinder: Precompute must be called before Export")
	}
	magic := jpsPlusMagic8
	cellBytes := 1
	if j.wide {
		magic = jpsPlusMagic16
		cellBytes = 2
	}
	if j.canonical {
		magic |= jpsPlusCanonicalFlag
	}
	headerSize := 12
	if j.hasFingerprint {
		magic |= jpsPlusFingerprintFlag | jpsPlusChecksumFlag
		headerSize += 8
	}
	// Format: magic(4) + width(4) + height(4) + optional fingerprint(8)
	// + table + optional checksum(4).
	buf := make([]byte, j.PrecomputedSize())
	binary.LittleEndian.PutUint32(buf[0:4], magic)
	binary.LittleEndian.PutUint32(buf[4:8], uint32(j.width))
	binary.LittleEndian.PutUint32(buf[8:12], uint32(j.height))
	if j.hasFingerprint {
		binary.LittleEndian.PutUint64(buf[12:20], j.fingerprint)
	}
	if j.wide {
		for i, v := range j.jump16 {
			binary.LittleEndian.PutUint16(buf[headerSize+i*cellBytes:], uint16(v))
		}
	} else {
		for i, v := range j.jump8 {
			buf[headerSize+i] = byte(v)
		}
	}
	if j.hasFingerprint {
		digestOffset := len(buf) - 4
		binary.LittleEndian.PutUint32(buf[digestOffset:], crc32.ChecksumIEEE(buf[:digestOffset]))
	}
	return buf, nil
}

// LoadPrecomputed loads previously exported jump table data.
// FindPath verifies the dimensions and obstacle layout against the table.
// Canonical tables also require DiagonalAlways.
// A failed load leaves the previous table available.
// Legacy data without a fingerprint can be loaded, but searches use online
// JPS until Precompute builds a table bound to the map.
func (j *JPSPlusFinder) LoadPrecomputed(data []byte) error {
	table, err := parseJumpTable(data)
	if err != nil {
		return err
	}
	j.installJumpTable(table)
	return nil
}

// encodedJumpTable is a validated view of an input buffer. The finder
// copies the distances when installing it and never retains the input.
type encodedJumpTable struct {
	width, height   int32
	wide, canonical bool
	fingerprint     uint64
	hasFingerprint  bool
	payload         []byte
}

func (table encodedJumpTable) distance(index int) int16 {
	if table.wide {
		return int16(binary.LittleEndian.Uint16(table.payload[index*2:]))
	}
	return int16(int8(table.payload[index]))
}

func parseJumpTable(data []byte) (encodedJumpTable, error) {
	if len(data) < 12 {
		return encodedJumpTable{}, errors.New("JPSPlusFinder: precomputed data too short")
	}
	magic := binary.LittleEndian.Uint32(data[0:4])
	var wide bool
	switch magic &^ (jpsPlusCanonicalFlag | jpsPlusFingerprintFlag | jpsPlusChecksumFlag) {
	case jpsPlusMagic8:
	case jpsPlusMagic16:
		wide = true
	default:
		return encodedJumpTable{}, fmt.Errorf("JPSPlusFinder: invalid magic 0x%X", magic)
	}
	w := binary.LittleEndian.Uint32(data[4:8])
	h := binary.LittleEndian.Uint32(data[8:12])
	hasFingerprint := magic&jpsPlusFingerprintFlag != 0
	canonical := magic&jpsPlusCanonicalFlag != 0
	hasChecksum := magic&jpsPlusChecksumFlag != 0
	headerSize := 12
	var fingerprint uint64
	if hasFingerprint {
		headerSize += 8
		if len(data) < headerSize {
			return encodedJumpTable{}, errors.New("JPSPlusFinder: precomputed data is missing its fingerprint")
		}
		fingerprint = binary.LittleEndian.Uint64(data[12:20])
	}
	cellBytes := 1
	if wide {
		cellBytes = 2
	}
	cells := uint64(w) * uint64(h)
	if w == 0 || h == 0 || cells > (1<<31-1)/8 || cells > uint64((len(data)-headerSize)/(8*cellBytes)) {
		return encodedJumpTable{}, errors.New("JPSPlusFinder: invalid dimensions or truncated precomputed data")
	}
	tableSize := int(cells * 8)
	payloadEnd := headerSize + tableSize*cellBytes
	if hasChecksum {
		if len(data)-payloadEnd != 4 ||
			binary.LittleEndian.Uint32(data[payloadEnd:]) != crc32.ChecksumIEEE(data[:payloadEnd]) {
			return encodedJumpTable{}, errors.New("JPSPlusFinder: invalid precomputed data checksum")
		}
	}
	payload := data[headerSize:payloadEnd]
	table := encodedJumpTable{
		width: int32(w), height: int32(h), wide: wide, canonical: canonical,
		fingerprint: fingerprint, hasFingerprint: hasFingerprint, payload: payload,
	}
	if err := table.validateDistances(); err != nil {
		return encodedJumpTable{}, err
	}
	return table, nil
}

func (j *jumpTable) installJumpTable(table encodedJumpTable) {
	j.width, j.height = table.width, table.height
	j.wide, j.canonical = table.wide, table.canonical
	j.fingerprint, j.hasFingerprint = table.fingerprint, table.hasFingerprint
	j.resize()
	for i := range j.jump16 {
		j.jump16[i] = table.distance(i)
	}
	for i := range j.jump8 {
		j.jump8[i] = int8(table.distance(i))
	}
}

func (table encodedJumpTable) validateDistances() error {
	width, height := table.width, table.height
	for y := int32(0); y < height; y++ {
		for x := int32(0); x < width; x++ {
			base := int((y*width + x) * 8)
			for dir := dirN; dir <= dirNW; dir++ {
				distance := int32(table.distance(base + dir))
				// Legacy negative diagonal entries can include the first
				// blocked step. They are never used as jump destinations.
				if distance == 0 || (distance < 0 && !table.canonical) {
					continue
				}
				dx, dy := dirToVec(dir)
				endX, endY := x+AbsInt(distance)*dx, y+AbsInt(distance)*dy
				if endX < 0 || endX >= width || endY < 0 || endY >= height {
					return fmt.Errorf("JPSPlusFinder: jump distance outside the grid at (%d,%d), direction %d", x, y, dir)
				}
			}
		}
	}
	return nil
}

// PrecomputedSize returns the byte size of the serialized jump table.
// Useful for estimating storage or memory before exporting.
func (j *JPSPlusFinder) PrecomputedSize() int {
	if !j.hasTable() {
		return 0
	}
	overhead := 12
	if j.hasFingerprint {
		overhead += 8 + 4 // layout fingerprint and checksum
	}
	return overhead + len(j.jump8) + 2*len(j.jump16)
}
