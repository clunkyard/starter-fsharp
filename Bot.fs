// Narrow the scan around a contact and lead shots from recent sightings.
// Clunkyard guide: https://clunkyard.com/sdk/fsharp
module Bot
open System
open Clunkyard
let n (value:double)=Nullable value
let searchArc=40.0  // degrees: sweeps the circle in half a second
let holdWidth=60.0  // units across at the target: it stays in the beam from tick to tick
let minArc=6.0      // degrees: the narrowest the beam holds
let window=12       // ticks of sightings the aim averages
let sightings=8     // narrow sightings in the window before a shot
let lookEvery=40    // ticks between looks around the target, for a nearer robot
let reacquire=10    // ticks to look again where a lost target was
type Starter() =
    inherit Robot()
    let mutable free=false
    let mutable dodges=0      // dodges taken
    let mutable dodgeEnd=0    // the tick the current one ends
    let mutable slot=0        // the last target's slot, for looking again where it was lost
    member this.Steer(towards,current)=this.Angles.Clamp(this.Angles.AngleDiff(towards,current)/45.0)
    override this.OnStart()=free <- this.Match.Is3d && this.Arena.Gravity.Z=0.0
    // A straight line through the narrowest of the last `window` ticks' sightings: where the
    // robot is now and how fast it moves. Each sighting is off by its BearingError; the line averages that out.
    member this.Fit(slot:int,tick:int)=
        let recent=this.Tracker.History(slot)|>Seq.filter(fun s->s.Tick>tick-window)|>List.ofSeq
        if recent.Length<2 then None else
        let best=recent|>List.map(fun s->s.BearingError)|>List.min
        let seen=recent|>List.filter(fun s->s.BearingError<=1.5*best) // the narrow ones
        let count=seen.Length
        if count<2 then None else
        let t0=(seen|>List.sumBy(fun s->float s.Tick))/float count
        let spread=seen|>List.sumBy(fun s->(float s.Tick-t0)**2.0)
        // One axis: the line's position now and its slope per tick.
        let axis(of':Vec3->double)=
            let mean=(seen|>List.sumBy(fun s->of' s.Position))/float count
            let slope=(seen|>List.sumBy(fun s->(float s.Tick-t0)*(of' s.Position-mean)))/spread
            mean+slope*(float tick-t0),slope*float this.Match.TickRate
        let x,y,z=axis(fun p->p.X),axis(fun p->p.Y),axis(fun p->p.Z)
        Some(Vec3(fst x,fst y,fst z),Vec3(snd x,snd y,snd z),count)
    override this.OnTurn()=
        let self=this.Me
        let position=self.Position
        let gun,scanner=this.Gun,this.Scanner
        let tick=this.Clock.Tick
        this.Drive.Forces(brake=n 0.2)
        let target,cue=this.Sensors.Nearest,this.Tracker.Cue
        let lastSeen=if slot<>0 then this.Tracker.LastSeen(slot) else Nullable()
        if not(isNull target) then
            slot <- target.Slot
            let throttle=if target.Range>350.0 then 0.7 elif target.Range<200.0 then -0.4 else 0.2
            this.Drive.Set(n throttle,n(this.Steer(target.Bearing,self.Heading)),n(this.Steer(target.Elevation,self.Pitch)))
            // Halve the beam each tick until it is holdWidth across at the target:
            // narrow enough for a precise bearing, wide enough to keep it in.
            let hold=max minArc (2.0*Math.Atan2(holdWidth/2.0,target.Range)*180.0/Math.PI)
            // In a crowd a narrow beam sees one robot. Every lookEvery ticks,
            // widen it for a tick: a nearer robot it finds becomes the target.
            let arc=if tick%lookEvery=0 then searchArc else max hold (min searchArc (scanner.Last.Arc/2.0))
            scanner.Set(n target.Bearing,n arc,n target.Elevation)
            let power=gun.PowerForFlightTime(target.Range,1.0)
            // An exact contact: gun.Lead is the intercept if the target holds course.
            // A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
            let aim,ready=
                if target.BearingError>0.0 then
                    match this.Fit(target.Slot,tick) with
                    | Some(position,velocity,count)->gun.LeadPoint(position,velocity,power),count>=sightings
                    | None->Unchecked.defaultof<Solution>,false
                else gun.Lead(target,power),true
            let heading,pitch=if isNull aim then target.Bearing,target.Elevation else aim.Heading,aim.Pitch
            gun.Aim(n heading,n pitch)
            let now=gun.AimNow
            if ready && abs(this.Angles.AngleDiff(heading,now.Heading))<3.0 && abs(pitch-now.Pitch)<3.0 && gun.CanFire(power,10.0) then gun.Fire(n power)
            this.Status.Set((if ready then "TRACK" else "NARROW"),n target.Range)
        elif lastSeen.HasValue && tick-lastSeen.Value<=reacquire then
            // Lost it: look again where it was last seen, widening the beam, before searching.
            let where=match this.Tracker.Position(slot) with null->position | seen->seen
            let bearing=this.Angles.BearingTo(position,where)
            let elevation=if free then this.Angles.ElevationTo(position,where) else 0.0
            this.Drive.Set(n 0.5,n(this.Steer(bearing,self.Heading)),n(this.Steer(elevation,self.Pitch)))
            scanner.Set(n bearing,n(min searchArc (scanner.Last.Arc*2.0)),n elevation)
            this.Status.Set("REACQUIRE")
        elif not(isNull cue) then
            let bearing,distance=this.Angles.BearingTo(position,cue.Position),this.Angles.Distance(position,cue.Position)
            let elevation=if free then this.Angles.ElevationTo(position,cue.Position) else 0.0
            let error=min cue.BearingError 30.0
            let across=float(tick%5-2)/2.0*error
            let up=if free then float((tick/5)%3-1)*error/2.0 else 0.0
            this.Drive.Set(n 0.7,n(this.Steer(bearing,self.Heading)),n(this.Steer(elevation,self.Pitch)))
            scanner.Set(n(bearing+across),n(scanner.ArcForReach(distance+100.0)),n(elevation+up))
            this.Status.Set("CUE",n distance)
        else
            this.Drive.Set(n 0.5,n 0.33,n(this.Steer(0.0,self.Pitch)))
            let pitch=if free then float((tick/10)%3-1)*35.0 else 0.0
            scanner.Set(n(this.Angles.Normalize(scanner.Last.Heading+36.0)),n searchArc,n pitch)
            this.Status.Set("SEARCH")
        // A simple dodge: vary it, or predict the shooter.
        if tick>=dodgeEnd && this.Sensors.Cues |> Seq.exists(fun c->c.Source=Source.Muzzle) then
            dodges <- dodges+1
            dodgeEnd <- tick+10
        if tick<dodgeEnd then
            // Off the line of fire: left, then right next time (turn is positive left, strafe positive right).
            let left=dodges%2=1
            if this.Match.Control=Control.Forces then
                this.Drive.Set(throttle=n 0.5)
                this.Drive.Forces(strafe=n(if left then -1.0 else 1.0))
            else this.Drive.Set(n 1.0,n(if left then 1.0 else -1.0))
            this.Status.Set((if left then "dodge:left" else "dodge:right"),n(float dodges))
        if this.Battery.Charge<this.Battery.Capacity/4.0 then this.Drive.Set(throttle=n(this.Drive.Get("throttle").GetValueOrDefault(0.0)*0.5))
        let storm,zone=this.Nav.Storm,this.Nav.Zone
        if not(isNull storm) && storm.Margin(position).GetValueOrDefault(-1e9)> -60.0 then
            let away=this.Angles.BearingTo(storm.Center,position)
            this.Drive.Set(throttle=n 0.8,turn=n(this.Angles.Clamp(this.Angles.AngleDiff(away,self.Heading)/30.0)))
        if not(isNull zone) && zone.Margin(position).GetValueOrDefault(1e9)<60.0 then
            let climb=if zone.Shape=Shape.Sphere then this.Angles.ElevationTo(position,zone.Center) else 0.0
            this.Drive.Set(n 0.8,n(this.Angles.Clamp(this.Angles.AngleDiff(this.Angles.BearingTo(position,zone.Center),self.Heading)/30.0)),n(this.Angles.Clamp((climb-self.Pitch)/30.0)))
[<EntryPoint>]
let main _ =
    Clunkyard.Clunkyard.Run(Starter())
    0
