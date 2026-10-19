package grid

import (
	"errors"
	"fmt"
	"math/rand"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

func TestPoolManagerEvictsOnRelease(t *testing.T) {
	m := NewPoolManager()

	pr, _ := m.Acquire("map", smallFactory, WithCapacity(4))
	if pr == nil {
		t.Fatal("Acquire returned nil")
	}
	got, err := pr.FindPathWorld(0, 0, 7*32, 7*32)
	if err != nil {
		t.Fatalf("FindPathWorld: %v", err)
	}
	if len(got) == 0 {
		t.Fatalf("expected a path across an empty map")
	}
	pr.Release()
	pr.Release() // double release must be safe

	if m.Pool("map") != nil {
		t.Fatalf("pool must be evicted after the last scene released")
	}
}

func TestPoolManagerSharedPool(t *testing.T) {
	m := NewPoolManager()

	a, _ := m.Acquire("map", smallFactory, WithCapacity(4))
	b, _ := m.Acquire("map", smallFactory, WithCapacity(4))
	if m.Pool("map") == nil {
		t.Fatal("expected a shared pool while scenes hold it")
	}

	a.Release()
	if m.Pool("map") == nil {
		t.Fatal("pool must survive while another scene holds it")
	}
	got, err := b.FindPathWorld(0, 0, 7*32, 7*32)
	if err != nil {
		t.Fatalf("FindPathWorld: %v", err)
	}
	if len(got) == 0 {
		t.Fatalf("expected a path from the remaining scene")
	}
	b.Release()

	if m.Pool("map") != nil {
		t.Fatal("pool must be evicted once every scene released")
	}
}

func TestPoolManagerKeysAreIsolated(t *testing.T) {
	m := NewPoolManager()

	a, _ := m.Acquire("mapA", smallFactory, WithCapacity(4))
	defer a.Release()
	b, _ := m.Acquire("mapB", smallFactory, WithCapacity(4))

	a.Release()
	if m.Pool("mapB") == nil {
		t.Fatal("releasing one key must not evict another")
	}
	if m.Pool("mapA") != nil {
		t.Fatal("released key must be evicted")
	}
	b.Release()
}

func TestPoolManagerConcurrent(t *testing.T) {
	m := NewPoolManager()
	var wg sync.WaitGroup
	for i := 0; i < 64; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < 50; j++ {
				pr, _ := m.Acquire("map", smallFactory, WithCapacity(4))
				_, _ = pr.FindPathWorld(0, 0, 7*32, 7*32)
				pr.Release()
			}
		}()
	}
	wg.Wait()

	if m.Pool("map") != nil {
		t.Fatalf("pool must be evicted after all scenes released")
	}
}

// Two Proxies share a Pool, so they can be handed the same leased instance.
// Each result must survive the other Proxy searching with it, which is what the
// Proxy-owned result buffer is for.
func TestPoolManagerProxyResultsAreIsolated(t *testing.T) {
	m := NewPoolManager()
	a, _ := m.Acquire("map", smallFactory, WithCapacity(1)) // capacity 1: both proxies contend for one instance
	b, _ := m.Acquire("map", smallFactory, WithCapacity(1))
	defer a.Release()
	defer b.Release()

	first, _ := a.FindPathWorld(0, 0, 7*32, 7*32)
	if len(first) == 0 {
		t.Fatal("expected a path from the first proxy")
	}
	want := append([][2]float32(nil), first...)

	// b searches using the instance a just returned to the pool.
	_, _ = b.FindPathWorld(7*32, 7*32, 0, 0)

	for i, p := range want {
		if first[i] != p {
			t.Fatalf("result of the first proxy was overwritten at %d: got %v want %v", i, first[i], p)
		}
	}
}

func TestProxyReusesResultBuffer(t *testing.T) {
	m := NewPoolManager()
	pr, _ := m.Acquire("map", smallFactory, WithCapacity(4))
	defer pr.Release()

	first, _ := pr.FindPathWorld(0, 0, 7*32, 7*32)
	if len(first) == 0 {
		t.Fatal("expected a path")
	}
	second, _ := pr.FindPathWorld(7*32, 7*32, 0, 0)
	if len(second) == 0 {
		t.Fatal("expected a path")
	}
	if &first[0] != &second[0] {
		t.Fatal("expected the proxy to reuse its result buffer across calls")
	}
}

// blockedFactory makes an 8x8 grid with every tile an obstacle.
func blockedFactory() Grid {
	m := NewMatrix(8, 8)
	for y := range m {
		for x := range m[y] {
			m[y][x] = 1
		}
	}
	return NewOrthogonalGrid(m, WithTileSize(32, 32))
}

// mustPanic runs fn and fails unless it panics with a message mentioning want.
func mustPanic(t *testing.T, want string, fn func()) {
	t.Helper()
	defer func() {
		r := recover()
		if r == nil {
			t.Fatalf("expected a panic mentioning %q, got none", want)
		}
		if msg := fmt.Sprint(r); !strings.Contains(msg, want) {
			t.Fatalf("panic message %q does not mention %q", msg, want)
		}
	}()
	fn()
}

func TestPoolManagerResetSwapsPool(t *testing.T) {
	m := NewPoolManager()
	pr, _ := m.Acquire("map", smallFactory, WithCapacity(4))

	got, err := pr.FindPathWorld(0, 0, 7*32, 7*32)
	if err != nil {
		t.Fatalf("FindPathWorld: %v", err)
	}
	if len(got) == 0 {
		t.Fatal("expected a path before the reset")
	}
	if got := pr.ObstacleCount(); got != 0 {
		t.Fatalf("expected the open map, obstacleCount=%d want 0", got)
	}

	// The map changed: the replacement carries a fully blocked grid.
	if err := m.Reset("map", blockedFactory); err != nil {
		t.Fatalf("Reset on a live pool: %v", err)
	}
	if m.Pool("map") == nil {
		t.Fatal("Reset must install the replacement pool straight away")
	}
	if m.Pool("map").Closed() {
		t.Fatal("the replacement pool must be live")
	}

	// The very same Proxy keeps working: Reset changes map data, nothing else.
	if got := pr.ObstacleCount(); got != 64 {
		t.Fatalf("proxy must serve the new map, obstacleCount=%d want 64", got)
	}
	if _, err := pr.FindPathWorld(0, 0, 7*32, 7*32); err != nil {
		t.Fatalf("proxy must still search after Reset: %v", err)
	}

	// A scene that never heard about the change and passes the stale factory
	// must still land on the new data rather than resurrecting the old map.
	stale, _ := m.Acquire("map", smallFactory, WithCapacity(4))
	if got := stale.ObstacleCount(); got != 64 {
		t.Fatalf("Reset's factory must win over Acquire's, obstacleCount=%d want 64", got)
	}

	// Releasing both scenes still evicts the pool as usual.
	if m.Pool("map") == nil {
		t.Fatal("pool must stay registered while a scene holds it")
	}
	stale.Release()
	pr.Release()
	if m.Pool("map") != nil {
		t.Fatal("pool must be evicted once every scene released")
	}
}

// A released Proxy is dead: every Grid method has to panic instead of quietly
// serving a map that is gone.
func TestProxyPanicsAfterRelease(t *testing.T) {
	m := NewPoolManager()
	pr, _ := m.Acquire("map", smallFactory, WithCapacity(4))

	// Sanity: everything works while the proxy is live.
	_ = pr.Width()
	_ = pr.IsWalkableAt(0, 0)
	_ = pr.ObstacleCount()
	_, _ = pr.WorldToTile(0, 0)
	_ = pr.HasLineOfSight(0, 0, 1, 1)
	_ = pr.Finder()
	_, _ = pr.FindPath(0, 0, 7, 7)
	_, _ = pr.FindPathWorld(0, 0, 7*32, 7*32)

	pr.Release()
	pr.Release() // double release stays safe

	calls := map[string]func(){
		"Width":         func() { _ = pr.Width() },
		"Height":        func() { _ = pr.Height() },
		"IsInside":      func() { _ = pr.IsInside(0, 0) },
		"IsWalkableAt":  func() { _ = pr.IsWalkableAt(0, 0) },
		"SetWalkableAt": func() { pr.SetWalkableAt(0, 0, false) },
		"GetNodeAt":     func() { _ = pr.GetNodeAt(0, 0) },
		"Clone":         func() { _ = pr.Clone() },
		"TileWidth":     func() { _ = pr.TileWidth() },
		"TileHeight":    func() { _ = pr.TileHeight() },
		"ObstacleCount": func() { _ = pr.ObstacleCount() },
		"TileIndex":     func() { _ = pr.TileIndex(0, 0) },
		"TileXY":        func() { _, _ = pr.TileXY(0) },
		"SetWeightAt":   func() { pr.SetWeightAt(0, 0, 2) },
		"GetWeightAt":   func() { _ = pr.GetWeightAt(0, 0) },
		"WorldToTile":   func() { _, _ = pr.WorldToTile(0, 0) },
		"TileToWorld":   func() { _, _ = pr.TileToWorld(0, 0) },
		"IsWalkableAtWorld": func() {
			_ = pr.IsWalkableAtWorld(0, 0)
		},
		"SetWalkableAtWorld": func() { pr.SetWalkableAtWorld(0, 0, false) },
		"HasLineOfSight":     func() { _ = pr.HasLineOfSight(0, 0, 1, 1) },
		"HasLineOfSightWorld": func() {
			_ = pr.HasLineOfSightWorld(0, 0, 1, 1)
		},
		"FindNearestWalkable": func() { _, _, _ = pr.FindNearestWalkable(0, 0, 2) },
		"FindNearestWalkableWorld": func() {
			_, _, _ = pr.FindNearestWalkableWorld(0, 0, 2, 1)
		},
		"Finder":        func() { _ = pr.Finder() },
		"RenderSVG":     func() { _ = pr.RenderSVG(nil) },
		"FindPath":      func() { _, _ = pr.FindPath(0, 0, 7, 7) },
		"FindPathWorld": func() { _, _ = pr.FindPathWorld(0, 0, 7*32, 7*32) },
		"GetNeighbors": func() {
			_ = pr.GetNeighbors(pr.GetNodeAt(0, 0), 0, nil)
		},
	}
	for name, call := range calls {
		mustPanic(t, "released Proxy", func() { call() })
		_ = name
	}
}

func TestPoolManagerResetPicksUpNewFactory(t *testing.T) {
	m := NewPoolManager()
	// Keep a second scene on the key so the Pool survives to be Reset.
	keeper, _ := m.Acquire("map", blockedFactory, WithCapacity(4))
	defer keeper.Release()

	pr, _ := m.Acquire("map", blockedFactory, WithCapacity(4))
	if got := pr.ObstacleCount(); got != 64 {
		t.Fatalf("expected obstacleCount=64, got %d", got)
	}

	if err := m.Reset("map", smallFactory); err != nil {
		t.Fatalf("Reset: %v", err)
	}

	// Same key, different factory: the fresh pool must use the new one.
	fresh, _ := m.Acquire("map", blockedFactory, WithCapacity(4))
	defer fresh.Release()
	if got := fresh.ObstacleCount(); got != 0 {
		t.Fatalf("expected the new factory's open map, obstacleCount=%d want 0", got)
	}
}

func TestPoolManagerResetLeavesOtherKeys(t *testing.T) {
	m := NewPoolManager()
	a, _ := m.Acquire("mapA", smallFactory, WithCapacity(4))
	b, _ := m.Acquire("mapB", smallFactory, WithCapacity(4))

	if err := m.Reset("mapA", smallFactory); err != nil {
		t.Fatalf("Reset mapA: %v", err)
	}
	if m.Pool("mapB") == nil {
		t.Fatal("resetting one key must not touch another")
	}
	got, err := b.FindPathWorld(0, 0, 7*32, 7*32)
	if err != nil {
		t.Fatalf("FindPathWorld: %v", err)
	}
	if len(got) == 0 {
		t.Fatal("mapB must keep working")
	}

	a.Release()
	b.Release()
	if m.Pool("mapB") != nil {
		t.Fatal("mapB must be evicted once released")
	}
}

// Scenes that survive a Reset by re-Acquiring must still let the pool be
// evicted, i.e. the discarded pool's references must not linger.
func TestPoolManagerResetNoRefCountLeak(t *testing.T) {
	m := NewPoolManager()

	var held []Proxy
	for i := 0; i < 5; i++ {
		pr, err := m.Acquire("map", smallFactory, WithCapacity(4))
		if err != nil {
			t.Fatal(err)
		}
		held = append(held, pr)
	}
	_ = m.Reset("map", smallFactory)

	for _, pr := range held {
		pr.Release() // stale releases must be dropped, not counted
	}
	for i := 0; i < 5; i++ {
		pr, err := m.Acquire("map", smallFactory, WithCapacity(4))
		if err != nil {
			t.Fatal(err)
		}
		held = append(held, pr)
	}
	for _, pr := range held {
		pr.Release()
	}

	if m.Pool("map") != nil {
		t.Fatal("pool must be evicted after every scene released")
	}
}

// Reset must be safe to call while scenes are mid-search: nothing is
// invalidated, so no recover is expected and no request may fail for a reason the
// caller could not act on.
func TestPoolManagerResetConcurrent(t *testing.T) {
	m := NewPoolManager()
	var wg sync.WaitGroup

	for i := 0; i < 16; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < 100; j++ {
				pr, _ := m.Acquire("map", smallFactory, WithCapacity(4))
				// A Reset landing mid-search panics by design; the scene
				// recovers and re-Acquires, which is the documented contract.
				func() {
					defer func() { recover() }()
					_, _ = pr.FindPathWorld(0, 0, 7*32, 7*32)
				}()
				pr.Release()
			}
		}()
	}

	wg.Add(1)
	go func() {
		defer wg.Done()
		for j := 0; j < 50; j++ {
			_ = m.Reset("map", smallFactory)
		}
	}()

	wg.Wait()
	_ = m.Reset("map", smallFactory)

	if m.Pool("map") != nil {
		t.Fatal("pool must be evictable after the churn settles")
	}
}

// The PoolManager must evict a Pool exactly when its last holder lets go: no early
// eviction (a live Proxy pointing at a closed Pool) and no late eviction (a dead
// Pool left registered).
func TestPoolManagerRefCountExact(t *testing.T) {
	m := NewPoolManager()

	const scenes = 10
	held := make([]Proxy, 0, scenes)
	for i := 0; i < scenes; i++ {
		pr, err := m.Acquire("map", smallFactory, WithCapacity(4))
		if err != nil {
			t.Fatal(err)
		}
		held = append(held, pr)
		if m.Pool("map") == nil {
			t.Fatalf("pool evicted after only %d of %d scenes held it", i+1, scenes)
		}
	}

	for i := 0; i < scenes-1; i++ {
		held[i].Release()
		if m.Pool("map") == nil {
			t.Fatalf("pool evicted while %d scenes still held it", scenes-i-1)
		}
	}

	held[scenes-1].Release()
	if m.Pool("map") != nil {
		t.Fatal("pool must be evicted once the last scene released")
	}
}

// Randomised Acquire/Release/Reset churn. Whatever the interleaving, the map
// must end up empty once the dust settles — a lost reference would keep a Pool
// registered forever.
func TestPoolManagerChurnSettles(t *testing.T) {
	m := NewPoolManager()
	rng := rand.New(rand.NewSource(1))

	var live []Proxy
	for step := 0; step < 4000; step++ {
		switch rng.Intn(3) {
		case 0, 1:
			pr, err := m.Acquire("map", smallFactory, WithCapacity(4))
			if err != nil {
				t.Fatal(err)
			}
			live = append(live, pr)
		case 2:
			if len(live) > 0 {
				i := rng.Intn(len(live))
				live[i].Release()
				live = append(live[:i], live[i+1:]...)
			}
		}
		if step%97 == 0 {
			_ = m.Reset("map", smallFactory)
			// A discarded Proxy is dead, so scenes let go of theirs.
			for _, pr := range live {
				pr.Release()
			}
			live = live[:0]
		}
		for _, pr := range live {
			_, _ = pr.FindPathWorld(0, 0, 7*32, 7*32)
		}
	}

	for _, pr := range live {
		pr.Release()
	}
	_ = m.Reset("map", smallFactory)
	if m.Pool("map") != nil {
		t.Fatal("a lost reference kept the pool registered")
	}
}

func BenchmarkProxyFindPathWorld(b *testing.B) {
	m := NewPoolManager()
	pr, _ := m.Acquire("bench", func() Grid {
		return NewOrthogonalGrid(generateGrid(24, 24, 0.2), WithTileSize(32, 32))
	}, WithCapacity(4))
	defer pr.Release()

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, _ = pr.FindPathWorld(0, 0, 23*32, 23*32)
	}
}

// A nil Factory must come back as an error, not a panic: this library is
// embedded in a host process, and a configuration that turns up nil at runtime
// should be reportable rather than fatal.
func TestPoolManagerRejectsNilFactory(t *testing.T) {
	m := NewPoolManager()

	pr, err := m.Acquire("map", nil)
	if !errors.Is(err, ErrNilFactory) {
		t.Fatalf("Acquire with nil factory: err = %v, want ErrNilFactory", err)
	}
	if pr != nil {
		t.Fatal("a failed Acquire must not return a Proxy")
	}
	if m.Pool("map") != nil {
		t.Fatal("a failed Acquire must not register a Pool")
	}

	if err := m.Reset("map", nil); !errors.Is(err, ErrNilFactory) {
		t.Fatalf("Reset with nil factory: err = %v, want ErrNilFactory", err)
	}
}

// Reset must build its replacement before swapping, so a rejected configuration
// leaves the existing Pool serving traffic instead of dropping the key.
func TestPoolManagerResetFailureKeepsOldPool(t *testing.T) {
	m := NewPoolManager()
	pr, err := m.Acquire("map", smallFactory, WithCapacity(2))
	if err != nil {
		t.Fatal(err)
	}
	defer pr.Release()
	before := m.Pool("map")

	if err := m.Reset("map", nil); err == nil {
		t.Fatal("Reset with nil factory must fail")
	}
	after := m.Pool("map")
	if after != before {
		t.Fatal("a failed Reset must leave the original Pool registered")
	}
	if after == nil || after.Closed() {
		t.Fatal("the surviving Pool must stay usable")
	}
	if got, err := pr.FindPath(0, 0, 7, 7); err != nil || len(got) == 0 {
		t.Fatalf("the surviving Proxy must still search: len=%d err=%v", len(got), err)
	}
}

// Resetting a key nobody holds must say so. Reporting success it did not
// deliver is how a stale Factory silently keeps building the old map.
func TestPoolManagerResetUnheldKeyReportsErrNoPool(t *testing.T) {
	m := NewPoolManager()
	err := m.Reset("nothing", smallFactory)
	if !errors.Is(err, ErrNoPool) {
		t.Fatalf("Reset of an unheld key = %v, want ErrNoPool", err)
	}
	if m.Pool("nothing") != nil {
		t.Fatal("Reset must not create a Pool for an unheld key")
	}
}

// Concurrent scene churn plus search traffic plus a stats observer. Nothing
// should panic, the snapshots must stay self-consistent, and the Pool must
// settle back to empty once every scene lets go.
func TestPoolManagerChurnUnderTraffic(t *testing.T) {
	for trial := 0; trial < 60; trial++ {
		rng := rand.New(rand.NewSource(int64(trial)))
		m := NewPoolManager()
		timeout := time.Duration(200+rng.Intn(3000)) * time.Microsecond
		capacity := 1 + rng.Intn(4)

		var wg sync.WaitGroup
		var stop atomic.Bool

		for w := 0; w < 6; w++ {
			wg.Add(1)
			go func() {
				defer wg.Done()
				for !stop.Load() {
					pr, err := m.Acquire("k", smallFactory, WithCapacity(capacity), WithAcquireTimeout(timeout))
					if err != nil {
						continue
					}
					if _, err := pr.FindPath(0, 0, 7, 7); err != nil &&
						err != ErrPoolTimeout && err != ErrPoolClosed {
						t.Errorf("unexpected search error: %v", err)
					}
					pr.Release()
				}
			}()
		}
		wg.Add(1)
		go func() {
			defer wg.Done()
			for !stop.Load() {
				if pl := m.Pool("k"); pl != nil {
					s := pl.Stats()
					if s.Instances < 0 || s.Idle < 0 || s.Instances > s.Capacity || s.Idle > s.Instances {
						t.Errorf("trial %d: inconsistent stats %+v", trial, s)
					}
				}
			}
		}()

		time.Sleep(2 * time.Millisecond)
		stop.Store(true)
		wg.Wait()

		if pl := m.Pool("k"); pl != nil {
			s := pl.Stats()
			if s.Scenes != 0 || s.Instances != 0 || s.Idle != 0 {
				t.Fatalf("trial %d: pool not settled: %+v", trial, s)
			}
		}
	}
}

// Reset keeps the Pool, its scene references and every live Proxy, and only the
// map data changes. Releasing afterwards must still evict the Pool normally.
func TestPoolManagerResetReplacesAfterScenesHoldIt(t *testing.T) {
	for trial := 0; trial < 60; trial++ {
		rng := rand.New(rand.NewSource(int64(trial)))
		m := NewPoolManager()
		capacity := 1 + rng.Intn(4)

		var held []Proxy
		for i := 0; i < 1+rng.Intn(4); i++ {
			pr, err := m.Acquire("k", smallFactory, WithCapacity(capacity))
			if err != nil {
				t.Fatal(err)
			}
			held = append(held, pr)
		}
		before := m.Pool("k")
		scenesBefore := before.Stats().Scenes

		for _, pr := range held {
			if _, err := pr.FindPath(0, 0, 7, 7); err != nil {
				t.Fatalf("trial %d: search before reset: %v", trial, err)
			}
		}

		// Reset while scenes still hold it.
		if err := m.Reset("k", blockedFactory); err != nil {
			t.Fatalf("trial %d: reset: %v", trial, err)
		}

		// The Pool object is the same one and still counts every scene.
		after := m.Pool("k")
		if after != before {
			t.Fatalf("trial %d: Reset must keep the same Pool", trial)
		}
		if got := after.Stats().Scenes; got != scenesBefore {
			t.Fatalf("trial %d: Reset changed the scene count: %d -> %d", trial, scenesBefore, got)
		}

		// Every Proxy held before the Reset keeps working, on the new data.
		for _, pr := range held {
			if got := pr.ObstacleCount(); got != 64 {
				t.Fatalf("trial %d: proxy must serve the new map, obstacleCount=%d", trial, got)
			}
			if _, err := pr.FindPath(0, 0, 7, 7); err != nil {
				t.Fatalf("trial %d: search after reset: %v", trial, err)
			}
		}

		for _, pr := range held {
			pr.Release()
		}
		if m.Pool("k") != nil {
			t.Fatalf("trial %d: pool must be evicted once every scene released", trial)
		}

		fresh, err := m.Acquire("k", smallFactory, WithCapacity(capacity))
		if err != nil {
			t.Fatalf("trial %d: acquire after reset: %v", trial, err)
		}
		if p, err := fresh.FindPath(0, 0, 7, 7); err != nil || len(p) == 0 {
			t.Fatalf("trial %d: search after reset: len=%d err=%v", trial, len(p), err)
		}
		fresh.Release()
		if m.Pool("k") != nil {
			t.Fatalf("trial %d: pool must be evicted after everyone released", trial)
		}
	}
}

// The exact scenario that used to crash: Reset landing while scenes are
// mid-search used to panic with "Proxy whose Pool was discarded by Reset".
//
// Each worker takes its own Proxy — the documented shape, one goroutine per Proxy
// — so this is N scenes searching the same map while Reset swaps it underneath.
func TestPoolManagerResetDuringSearchNeverPanics(t *testing.T) {
	// A map big enough that a search takes real time and overlaps Reset.
	terrain := make([][]int, 300)
	for y := range terrain {
		terrain[y] = make([]int, 300)
		for x := range terrain[y] {
			terrain[y][x] = 0
		}
	}
	for y := 150; y < 300; y++ {
		for x := 0; x < 150; x++ {
			terrain[y][x] = 1
		}
	}
	factory := func() Grid { return NewOrthogonalGrid(terrain, WithTileSize(32, 32)) }

	for round := 0; round < 8; round++ {
		m := NewPoolManager()
		opts := []PoolOption{WithCapacity(4), WithAcquireTimeout(2 * time.Second)}

		var wg sync.WaitGroup
		for i := 0; i < 4; i++ {
			wg.Add(1)
			go func() {
				defer wg.Done()
				for j := 0; j < 6; j++ {
					pr, err := m.Acquire("town", factory, opts...)
					if err != nil {
						t.Errorf("round %d: acquire: %v", round, err)
						return
					}
					// read-only calls must survive too, not just searches
					_ = pr.Width()
					_ = pr.IsWalkableAt(10, 10)
					if _, err := pr.FindPathWorld(0, 0, 299*32, 299*32); err != nil {
						t.Errorf("round %d: search failed: %v", round, err)
					}
					pr.Release()
				}
			}()
		}
		for i := 0; i < 3; i++ {
			wg.Add(1)
			go func() {
				defer wg.Done()
				for j := 0; j < 12; j++ {
					if err := m.Reset("town", factory); err != nil && err != ErrNoPool {
						t.Errorf("round %d: reset: %v", round, err)
					}
				}
			}()
		}

		done := make(chan struct{})
		go func() { wg.Wait(); close(done) }()
		select {
		case <-done:
		case <-time.After(20 * time.Second):
			t.Fatalf("round %d: DEADLOCK", round)
		}
		if m.Pool("town") != nil {
			t.Fatalf("round %d: pool must be evicted once every scene released", round)
		}
	}
}
