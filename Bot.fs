// A robot on the arena's shared .NET core binding.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
module Bot
open System
open Clunkyard
let n (value:double)=Nullable value
// The calibration kernel, xorshift-sqrt v1: work rounds of a 64-bit
// xorshift and a square-root sum.
let calibrationWork (work: int64) (tick: uint64) =
    let mutable x = 88172645463325252UL ^^^ tick
    let mutable sum = 0.0
    for _ in 1L .. work do
        x <- x ^^^ (x <<< 13)
        x <- x ^^^ (x >>> 7)
        x <- x ^^^ (x <<< 17)
        sum <- sum + sqrt (float (x >>> 11))
    (uint64 (floor sum) + (x >>> 40)) % 1000000UL


type Starter() =
    inherit Robot()
    let mutable free=false
    member this.Steer(towards,current)=this.Angles.Clamp(this.Angles.AngleDiff(towards,current)/45.0)
    override this.OnStart()=free <- this.Match.Is3d && this.Arena.Gravity.Z=0.0
    override this.OnTurn()=
        let self=this.Me
        let position=self.Position
        let gun,scanner=this.Gun,this.Scanner
        this.Drive.Forces(brake=n 0.2)
        let target,cue=this.Sensors.Nearest,this.Tracker.Cue
        if not(isNull target) then
            let throttle=if target.Range>350.0 then 0.7 elif target.Range<200.0 then -0.4 else 0.2
            this.Drive.Set(n throttle,n(this.Steer(target.Bearing,self.Heading)),n(this.Steer(target.Elevation,self.Pitch)))
            scanner.Set(n target.Bearing,n 20.0,n target.Elevation)
            let power=gun.PowerForFlightTime(target.Range,1.0)
            let aim=gun.Lead(target,power)
            let heading,pitch=if isNull aim then target.Bearing,target.Elevation else aim.Heading,aim.Pitch
            gun.Aim(n heading,n pitch)
            let now=gun.AimNow
            if abs(this.Angles.AngleDiff(heading,now.Heading))<3.0 && abs(pitch-now.Pitch)<3.0 && gun.CanFire(power,10.0) then gun.Fire(n power)
            this.Status.Set("TRACK",n target.Range)
        elif not(isNull cue) then
            let bearing,distance=this.Angles.BearingTo(position,cue.Position),this.Angles.Distance(position,cue.Position)
            let elevation=if free then this.Angles.ElevationTo(position,cue.Position) else 0.0
            let error,tick=min cue.BearingError 30.0,this.Clock.Tick
            let across=float(tick%5-2)/2.0*error
            let up=if free then float((tick/5)%3-1)*error/2.0 else 0.0
            this.Drive.Set(n 0.7,n(this.Steer(bearing,self.Heading)),n(this.Steer(elevation,self.Pitch)))
            scanner.Set(n(bearing+across),n(scanner.ArcForReach(distance+100.0)),n(elevation+up))
            this.Status.Set("CUE",n distance)
        else
            this.Drive.Set(n 0.5,n 0.33,n(this.Steer(0.0,self.Pitch)))
            let pitch=if free then float((this.Clock.Tick/10)%3-1)*35.0 else 0.0
            scanner.Set(n(this.Angles.Normalize(scanner.Last.Heading+36.0)),n 40.0,n pitch)
            this.Status.Set("SEARCH")
        if this.Battery.Charge<this.Battery.Capacity/4.0 then this.Drive.Set(throttle=n(this.Drive.Get("throttle").GetValueOrDefault(0.0)*0.5))
        let storm,zone=this.Nav.Storm,this.Nav.Zone
        if not(isNull storm) && storm.Margin(position).GetValueOrDefault(-1e9)> -60.0 then
            let away=this.Angles.BearingTo(storm.Center,position)
            this.Drive.Set(throttle=n 0.8,turn=n(this.Angles.Clamp(this.Angles.AngleDiff(away,self.Heading)/30.0)))
        if not(isNull zone) && zone.Margin(position).GetValueOrDefault(1e9)<60.0 then
            let climb=if zone.Shape=Shape.Sphere then this.Angles.ElevationTo(position,zone.Center) else 0.0
            this.Drive.Set(n 0.8,n(this.Angles.Clamp(this.Angles.AngleDiff(this.Angles.BearingTo(position,zone.Center),self.Heading)/30.0)),n(this.Angles.Clamp((climb-self.Pitch)/30.0)))
        // The arena's rate calibration only; safe to delete.
        let work=int64(this.Raw.Number("state.calibration.work").GetValueOrDefault(0.0))
        if work>0L then this.Status.Set("WORK",n(float(calibrationWork work (uint64 this.Clock.Tick))))

[<EntryPoint>]
let main _ =
    Clunkyard.Clunkyard.Run(Starter())
    0
