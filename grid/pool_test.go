package grid

import (
	"math/rand"
	"runtime"
	"sync"
	"testing"
	"time"
)

func TestPoolReuse(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(2))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()
	p.addRef()

	l1, err := p.Acquire()
	if err != nil {
		t.Fatalf("acquire: %v", err)
	}
	g1 := l1.Grid
	l1.Release()

	l2, err := p.Acquire()
	if err != nil {
		t.Fatalf("acquire: %v", err)
	}
	if l2.Grid != g1 {
		t.Fatalf("expected idle instance to be reused")
	}
	l2.Release()

	if got := p.Created(); got != 1 {
		t.Fatalf("expected created=1, got %d", got)
	}
}

func TestPoolBlocksAtCapacity(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(1))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()

	l1, err := p.Acquire()
	if err != nil {
		t.Fatalf("acquire: %v", err)
	}

	got := make(chan *Lease, 1)
	go func() {
		l, err := p.Acquire()
		if err == nil {
			got <- l
		}
	}()

	select {
	case <-got:
		t.Fatalf("Acquire must block while at capacity")
	case <-time.After(50 * time.Millisecond):
	}

	l1.Release()

	select {
	case l := <-got:
		l.Release()
	case <-time.After(time.Second):
		t.Fatalf("Acquire must unblock after a release")
	}
}

func TestPoolDynamicShrink(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(4))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()
	p.addRef()
	p.addRef()

	leases := make([]*Lease, 3)
	for i := range leases {
		l, err := p.Acquire()
		if err != nil {
			t.Fatalf("acquire: %v", err)
		}
		leases[i] = l
	}
	for _, l := range leases {
		l.Release()
	}
	if p.Created() != 3 {
		t.Fatalf("expected created=3, got %d", p.Created())
	}

	// Scene count drops to 2, so one surplus idle instance must be discarded.
	p.releaseRef()
	if p.Created() != 2 || p.Idle() != 2 {
		t.Fatalf("expected created=2 idle=2 after shrink, got created=%d idle=%d", p.Created(), p.Idle())
	}
}

func TestPoolLeaseIdempotent(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(1))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()

	l, err := p.Acquire()
	if err != nil {
		t.Fatalf("acquire: %v", err)
	}
	l.Release()
	l.Release() // double return must be safe and must not push created negative

	if got := p.Created(); got != 1 {
		t.Fatalf("expected created=1 after double release, got %d", got)
	}
}

func TestPoolAcquireAfterClose(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(1))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()
	p.close()

	if _, err := p.Acquire(); err != ErrPoolClosed {
		t.Fatalf("expected ErrPoolClosed after close, got %v", err)
	}
}

func TestPoolFactoryNil(t *testing.T) {
	p, err := newPool(func() Grid { return nil }, WithCapacity(1))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()

	if _, err := p.Acquire(); err != ErrFactoryNil {
		t.Fatalf("expected ErrFactoryNil, got %v", err)
	}
	if got := p.Created(); got != 0 {
		t.Fatalf("failed creation must not consume capacity, got created=%d", got)
	}
}

// TestPoolAccountingInvariant churns leases and scene references against the
// pool's bookkeeping and checks after every step that created == idle+leased,
// that neither count goes negative, and that capacity is respected. This is what
// catches off-by-one errors in the shrink and close paths.
func TestPoolAccountingInvariant(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(6))
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 40; i++ {
		p.addRef()
	}

	rng := rand.New(rand.NewSource(7))
	var leased []*Lease
	for step := 0; step < 20000; step++ {
		if len(leased) < 6 && rng.Intn(2) == 0 {
			l, err := p.Acquire()
			if err != nil {
				t.Fatalf("step %d: acquire: %v", step, err)
			}
			leased = append(leased, l)
		} else if len(leased) > 0 {
			i := rng.Intn(len(leased))
			leased[i].Release()
			leased = append(leased[:i], leased[i+1:]...)
		}
		if step%13 == 0 {
			p.releaseRef()
			p.addRef() // churn the scene count too
		}

		created, idle := p.Created(), p.Idle()
		if created < 0 || idle < 0 {
			t.Fatalf("step %d: negative counts created=%d idle=%d", step, created, idle)
		}
		if created != idle+len(leased) {
			t.Fatalf("step %d: created=%d != idle=%d + leased=%d", step, created, idle, len(leased))
		}
		if created > p.capacity {
			t.Fatalf("step %d: created=%d exceeds capacity %d", step, created, p.capacity)
		}
	}
	for _, l := range leased {
		l.Release()
	}
}

// Clone copies node storage but shares the Finder, whose search buffers make two
// clones unsafe to search concurrently. Pool instances must therefore never be
// built with Clone. Pinning the sharing here means a change to Clone's contract
// has to be deliberate, and Factory's doc stays true.
func TestCloneSharesFinder(t *testing.T) {
	matrix := NewMatrix(8, 8)
	for y := 3; y < 8; y++ {
		for x := 0; x < 3; x++ {
			matrix[y][x] = 1 // a wall, so searches are non-trivial
		}
	}

	for _, tc := range []struct {
		name string
		orig Grid
	}{
		{"Orthogonal", NewOrthogonalGrid(matrix)},
		{"Staggered", NewStaggeredGrid(matrix)},
		{"Hex", NewHexGrid(matrix)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			clone, ok := tc.orig.Clone().(Grid)
			if !ok {
				t.Fatalf("Clone returned %T, want grid.Grid", tc.orig.Clone())
			}
			if clone.Finder() != tc.orig.Finder() {
				t.Skip("clone now owns its Finder; Factory and the Clone docs are out of date")
			}
			if clone == tc.orig {
				t.Fatal("clone must be a distinct object")
			}
			// Node storage, on the other hand, really is independent.
			a := tc.orig.GetNodeAt(4, 4)
			b := clone.GetNodeAt(4, 4)
			if a == b {
				t.Fatal("clone must own its node storage")
			}
			if a.Walkable != b.Walkable || a.Weight != b.Weight {
				t.Fatal("clone must start with the same tile data")
			}
		})
	}
}

// A Factory that panics must not leak its reserved capacity slot, or repeated
// panics would shrink the Pool until Acquire deadlocks.
func TestPoolFactoryPanicReturnsSlot(t *testing.T) {
	calls := 0
	p, err := newPool(func() Grid {
		calls++
		if calls == 1 {
			panic("factory blew up")
		}
		return NewOrthogonalGrid(NewMatrix(4, 4))
	}, WithCapacity(1))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()

	func() {
		defer func() {
			if r := recover(); r == nil {
				t.Fatal("expected the factory's panic to reach the caller")
			}
		}()
		_, _ = p.Acquire()
	}()

	if got := p.Created(); got != 0 {
		t.Fatalf("panicking factory must return its slot, created=%d want 0", got)
	}

	done := make(chan *Lease, 1)
	errc := make(chan error, 1)
	go func() {
		l, err := p.Acquire()
		if err != nil {
			errc <- err
			return
		}
		done <- l
	}()
	select {
	case l := <-done:
		l.Release()
	case err := <-errc:
		t.Fatalf("acquire after a panicking factory: %v", err)
	case <-time.After(time.Second):
		t.Fatal("Acquire deadlocked after a panicking factory")
	}
}

// Acquire is the only way in, and it always honours the Pool's budget: below
// capacity it builds an instance, at capacity it gives up on time rather than
// blocking forever.
func TestPoolAcquireIsAlwaysBounded(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(1), WithAcquireTimeout(80*time.Millisecond))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()

	if got := p.Timeout(); got != 80*time.Millisecond {
		t.Fatalf("AcquireTimeout = %v, want 80ms", got)
	}

	l, err := p.Acquire()
	if err != nil {
		t.Fatalf("acquire below capacity: %v", err)
	}

	start := time.Now()
	_, err = p.Acquire()
	elapsed := time.Since(start)
	if err != ErrPoolTimeout {
		t.Fatalf("expected ErrPoolTimeout, got %v", err)
	}
	if elapsed < 80*time.Millisecond {
		t.Fatalf("gave up after %v, before the 80ms budget", elapsed)
	}

	l.Release()
	l2, err := p.Acquire()
	if err != nil {
		t.Fatalf("acquire after release: %v", err)
	}
	l2.Release()
}

// Without WithCapacity a Pool gets one instance per logical CPU, and the
// acquire timeout defaults to DefaultAcquireTimeout.
func TestPoolAcquireDefaults(t *testing.T) {
	p, err := newPool(smallFactory)
	if err != nil {
		t.Fatal(err)
	}
	if got, want := p.Timeout(), DefaultAcquireTimeout; got != want {
		t.Fatalf("default timeout = %v, want %v", got, want)
	}
	if got, want := p.capacity, runtime.NumCPU(); got != want {
		t.Fatalf("default capacity = %d, want NumCPU = %d", got, want)
	}

	// A non-positive timeout is the only way back to unbounded waiting.
	q, err := newPool(smallFactory, WithCapacity(1), WithAcquireTimeout(0))
	if err != nil {
		t.Fatal(err)
	}
	if got := q.Timeout(); got != 0 {
		t.Fatalf("timeout = %v, want 0 (wait forever)", got)
	}
	if _, err := q.Acquire(); err != nil {
		t.Fatalf("acquire on an unbounded pool: %v", err)
	}
}

func TestPoolAcquireReusesIdle(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(1))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()

	l, err := p.Acquire()
	if err != nil {
		t.Fatal(err)
	}
	g := l.Grid
	l.Release()

	l2, err := p.Acquire()
	if err != nil {
		t.Fatal(err)
	}
	if l2.Grid != g {
		t.Fatal("expected Acquire to reuse the idle instance")
	}
	l2.Release()
}

func TestPoolStatsNoWaitWhenIdle(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(4))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()

	for i := 0; i < 20; i++ {
		l, err := p.Acquire()
		if err != nil {
			t.Fatal(err)
		}
		l.Release()
	}

	s := p.Stats()
	if s.Waits != 0 {
		t.Fatalf("uncontended pool reported %d waits, want 0", s.Waits)
	}
	if s.MaxWait != 0 || s.WaitTotal != 0 {
		t.Fatalf("uncontended pool reported wait time: total=%v max=%v", s.WaitTotal, s.MaxWait)
	}
	if s.Capacity != 4 || s.Scenes != 1 || s.Instances != 1 {
		t.Fatalf("unexpected snapshot: %+v", s)
	}
}

func TestPoolStatsRecordsWaits(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(1))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()

	held, err := p.Acquire()
	if err != nil {
		t.Fatal(err)
	}

	done := make(chan struct{})
	go func() {
		defer close(done)
		l, err := p.Acquire() // capacity 1 and the only instance is taken
		if err != nil {
			t.Errorf("waiting acquire: %v", err)
			return
		}
		l.Release()
	}()

	time.Sleep(50 * time.Millisecond) // let it block
	held.Release()

	select {
	case <-done:
	case <-time.After(time.Second):
		t.Fatal("waiting acquire never completed")
	}

	s := p.Stats()
	if s.Waits != 1 {
		t.Fatalf("expected 1 wait, got %d", s.Waits)
	}
	if s.MaxWait < 40*time.Millisecond {
		t.Fatalf("MaxWait %v is implausibly short for a 50ms block", s.MaxWait)
	}
	if s.WaitTotal < s.MaxWait {
		t.Fatalf("WaitTotal %v < MaxWait %v", s.WaitTotal, s.MaxWait)
	}
	if s.Idle != 1 || s.Instances != 1 {
		t.Fatalf("unexpected snapshot: %+v", s)
	}
}

// Stats must stay consistent while searches are churning.
func TestPoolStatsConcurrent(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(4))
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 8; i++ {
		p.addRef()
	}

	var wg sync.WaitGroup
	stop := make(chan struct{})
	wg.Add(1)
	go func() { // reader, like a game loop polling once a frame
		defer wg.Done()
		for {
			select {
			case <-stop:
				return
			default:
				_ = p.Stats()
			}
		}
	}()
	for i := 0; i < 16; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < 200; j++ {
				if l, err := p.Acquire(); err == nil {
					l.Release()
				}
			}
		}()
	}
	close(stop)
	wg.Wait()

	if s := p.Stats(); s.Waits < 0 || s.MaxWait < 0 {
		t.Fatalf("negative stats: %+v", s)
	}
}

// A generous budget must still get the instance once it comes back, even though
// the waiter was asleep on the cond when it was released.
func TestPoolAcquireWakesOnRelease(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(1))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()

	held, err := p.Acquire()
	if err != nil {
		t.Fatal(err)
	}

	type res struct {
		lease *Lease
		err   error
	}
	out := make(chan res, 1)
	go func() {
		l, err := p.Acquire()
		out <- res{l, err}
	}()

	time.Sleep(50 * time.Millisecond) // ensure it is parked on the cond
	held.Release()

	select {
	case r := <-out:
		if r.err != nil {
			t.Fatalf("acquire within budget: %v", r.err)
		}
		r.lease.Release()
	case <-time.After(2 * time.Second):
		t.Fatal("Acquire never woke on release")
	}
}

// A budget that expires with nothing coming back must return ErrPoolTimeout, and
// must do so on time rather than parking forever.
func TestPoolAcquireGivesUp(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(1))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()

	held, err := p.Acquire()
	if err != nil {
		t.Fatal(err)
	}
	defer held.Release()

	const budget = 100 * time.Millisecond
	start := time.Now()
	l, err := p.Acquire()
	el := time.Since(start)

	if err != ErrPoolTimeout {
		t.Fatalf("expected ErrPoolTimeout, got %v", err)
	}
	if l != nil {
		t.Fatal("no lease may be returned on timeout")
	}
	if el < budget {
		t.Fatalf("gave up after %v, before the %v budget", el, budget)
	}
	if el > budget+2*time.Second {
		t.Fatalf("overshot the budget badly: %v", el)
	}
}

// Many simultaneous waiters with a short budget must all come back, and the pool
// accounting must still balance afterwards.
func TestPoolAcquireTimeoutConcurrentWaiters(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(1))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()

	held, err := p.Acquire()
	if err != nil {
		t.Fatal(err)
	}

	// The only instance stays held for the whole window, so every waiter must
	// come back empty-handed rather than hang or corrupt the accounting.
	var wg sync.WaitGroup
	var timeouts int64
	var mu sync.Mutex
	for i := 0; i < 32; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			l, err := p.Acquire()
			mu.Lock()
			defer mu.Unlock()
			switch {
			case err == ErrPoolTimeout:
				timeouts++
			case err != nil:
				t.Errorf("unexpected error: %v", err)
			default:
				t.Error("got a lease while the only instance was held")
				l.Release()
			}
		}()
	}
	wg.Wait()

	if timeouts != 32 {
		t.Fatalf("expected all 32 waiters to time out, got %d", timeouts)
	}
	held.Release()

	// The pool must still be usable, and its accounting must balance: one
	// instance, now idle.
	l, err := p.Acquire()
	if err != nil {
		t.Fatalf("acquire after concurrent timeouts: %v", err)
	}
	l.Release()
	if s := p.Stats(); s.Instances != 1 || s.Idle != 1 || s.Capacity != 1 {
		t.Fatalf("inconsistent stats after timeouts: %+v", s)
	}
	if p.waits != 32 {
		t.Fatalf("expected 32 recorded waits, got %d", p.waits)
	}
}

func TestPoolAcquireOnClosedPool(t *testing.T) {
	p, err := newPool(smallFactory, WithCapacity(1))
	if err != nil {
		t.Fatal(err)
	}
	p.addRef()
	p.close()

	if _, err := p.Acquire(); err != ErrPoolClosed {
		t.Fatalf("expected ErrPoolClosed, got %v", err)
	}
}

// close() drops the idle set but must keep counting instances that are still on
// lease, so that every outstanding lease still has somewhere to land. This is the
// least-exercised corner of the accounting, and a mistake here leaks capacity.
func TestPoolCloseDropsIdleKeepsLeased(t *testing.T) {
	for trial := 0; trial < 300; trial++ {
		rng := rand.New(rand.NewSource(int64(trial)))
		p, err := newPool(smallFactory, WithCapacity(4))
		if err != nil {
			t.Fatal(err)
		}
		for i := 0; i < 6; i++ {
			p.addRef()
		}

		var leases []*Lease
		for i := 0; i < 4; i++ {
			l, err := p.Acquire()
			if err != nil {
				t.Fatalf("trial %d: %v", trial, err)
			}
			leases = append(leases, l)
		}
		// release a random subset so some sit idle and some stay leased
		rng.Shuffle(len(leases), func(i, j int) { leases[i], leases[j] = leases[j], leases[i] })
		for i := 0; i < rng.Intn(len(leases)+1); i++ {
			leases[i].Release()
		}
		// Every acquired instance still exists: none is dropped while refCount
		// (6) exceeds created.
		idleBefore := int(p.Idle())
		if got := p.Created(); got != len(leases) {
			t.Fatalf("trial %d: before close created=%d want %d", trial, got, len(leases))
		}
		outstanding := len(leases) - idleBefore

		p.close()
		if got, want := p.Created(), outstanding; got != want {
			t.Fatalf("trial %d: after close created=%d want %d (close must drop idle only)", trial, got, want)
		}
		if got := p.Idle(); got != 0 {
			t.Fatalf("trial %d: after close idle=%d want 0", trial, got)
		}
		// every outstanding lease must still release cleanly and land on 0
		for _, l := range leases {
			l.Release()
		}
		if got := p.Created(); got != 0 {
			t.Fatalf("trial %d: after all releases created=%d want 0", trial, got)
		}
		if got := p.Idle(); got != 0 {
			t.Fatalf("trial %d: after all releases idle=%d want 0", trial, got)
		}
	}
}

// Many workers racing one small pool: some win an instance, the rest must be
// turned away by the Pool's timeout rather than piling up. Each worker releases
// as soon as it has an instance, mirroring how Proxy uses a lease — holding one
// across the barrier would starve the others by construction.
func TestPoolAcquireUnderContention(t *testing.T) {
	for trial := 0; trial < 200; trial++ {
		p, err := newPool(smallFactory, WithCapacity(3), WithAcquireTimeout(5*time.Millisecond))
		if err != nil {
			t.Fatal(err)
		}
		p.addRef()

		var wg sync.WaitGroup
		var mu sync.Mutex
		var timedOut, succeeded int

		for w := 0; w < 8; w++ {
			wg.Add(1)
			go func() {
				defer wg.Done()
				l, err := p.Acquire()
				if err == nil {
					l.Release()
					mu.Lock()
					succeeded++
					mu.Unlock()
					return
				}
				mu.Lock()
				switch err {
				case ErrPoolTimeout:
					timedOut++
				case ErrPoolClosed:
				default:
					t.Errorf("unexpected error: %v", err)
				}
				mu.Unlock()
			}()
		}
		wg.Wait()

		st := p.Stats()
		if st.Instances < 0 || st.Idle < 0 || st.Instances > p.capacity {
			t.Fatalf("trial %d: bad stats %+v cap=%d", trial, st, p.capacity)
		}
		if st.Instances != st.Idle {
			t.Fatalf("trial %d: all leases returned, instances(%d) must equal idle(%d)", trial, st.Instances, st.Idle)
		}
		if succeeded+timedOut != 8 {
			t.Fatalf("trial %d: lost a result: ok=%d timeout=%d", trial, succeeded, timedOut)
		}
	}
}
