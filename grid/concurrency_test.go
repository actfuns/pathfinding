package grid

import (
	"sync"
	"testing"
)

// A released Proxy must not keep the map, Base, or buffers reachable.
func TestReleasedProxyDropsReferences(t *testing.T) {
	m := NewPoolManager()
	pr, err := m.Acquire("k", smallFactory, WithCapacity(2))
	if err != nil {
		t.Fatal(err)
	}
	if !pr.IsWalkableAt(0, 0) {
		t.Fatal("sanity: tile should be walkable")
	}
	p := pr.(*proxy)
	if p.base == nil {
		t.Fatal("expected Base to be cached")
	}
	pool := p.pool
	pr.Release()
	if p.base != nil || p.pool != nil || p.pathBuf != nil || p.worldBuf != nil {
		t.Fatal("released Proxy retained references")
	}
	if pool.base != nil {
		t.Fatal("closed Pool retained its Base grid")
	}
}

// Reset calls must not race with reads through live Proxies. Each Proxy stays
// on its own goroutine, per the Proxy contract.
func TestPoolManagerResetConcurrentReads(t *testing.T) {
	m := NewPoolManager()
	const scenes = 4
	proxies := make([]Proxy, 0, scenes)
	for i := 0; i < scenes; i++ {
		pr, err := m.Acquire("k", smallFactory, WithCapacity(scenes))
		if err != nil {
			t.Fatal(err)
		}
		proxies = append(proxies, pr)
	}
	defer func() {
		for _, pr := range proxies {
			pr.Release()
		}
	}()

	var wg sync.WaitGroup
	for i := 0; i < 32; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			_ = m.Reset("k", smallFactory)
		}()
	}
	for _, pr := range proxies {
		pr := pr
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < 100; j++ {
				_ = pr.IsWalkableAt(0, 0)
			}
		}()
	}
	wg.Wait()
}
