using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Ur16eInterfaces;

/// <summary>Object geometry in ROS metres, independently of XR tracking and TCP conventions.</summary>
public static class FixtureTilingGeometry
{
    public const string FrameId = "ur_base_link";

    public static bool TryBuildRequest(FixtureTilingSettings settings, string requestId,
        out FixtureTilingTaskMsg message, out string reason)
    {
        message = null;
        try
        {
            Require(settings != null, "Configure the tile batch first.");
            Require(ValidToken(requestId), "Use a new bounded ASCII request ID.");
            Require(Bounded(settings.ItemSize, 0, 2, false) &&
                Bounded(settings.ItemThickness, 0, settings.ItemSize, false), "Tile side and thickness must be positive; thickness cannot exceed the side.");
            Require(settings.SourceCount >= 1 && settings.SourceCount <= 64 && settings.ItemCount >= 1 &&
                settings.ItemCount <= settings.SourceCount && settings.Columns >= 1 && settings.Columns <= 64,
                "Use 1–64 source tiles and columns, and a job count no greater than the source count.");
            Require(Bounded(settings.StackPitch, 0, 2) &&
                (settings.StackPitch == 0 || settings.StackPitch >= settings.ItemThickness), "Stack pitch must be zero (tile thickness), or between thickness and 2 m.");
            Require(Bounded(settings.Gap, 0, 2) && Bounded(settings.RowOffsetFraction, 0, 1), "Gap or alternating row shift is outside its range.");
            Require(Finite(settings.ReleaseCenterX) && Finite(settings.ReleaseCenterY) &&
                Bounded(settings.ReleaseWidth, 0, 10) && Bounded(settings.ReleaseDepth, 0, 10) &&
                Bounded(settings.ReleaseClearance, 0, 1), "Release region and bottom clearance must be finite and within range.");
            Require(settings.ReleaseYawSamples >= 1 && settings.ReleaseYawSamples <= 12, "Use 1–12 release yaw samples.");
            Require(Bounded(settings.RetreatDistance, 0, 1, false) && Bounded(settings.SettlingDelay, 0, 60),
                "Retreat must be in (0,1] m and settling delay in [0,60] s.");
            Rotation source = ReadPose(settings.SourceTopPose);
            Rotation target = ReadPose(settings.PlacementCorner);
            Require((source.Z - new D3(0, 0, 1)).Length <= 1e-6, "Capture the centre of a horizontal stack top face; only stack yaw may vary.");
            Require(target.Z.Z > 1e-6, "The destination top-face normal must point upward.");
            message = new FixtureTilingTaskMsg {
                request_id = requestId, item_size = settings.ItemSize, item_thickness = settings.ItemThickness,
                source_top_pose = CopyPose(settings.SourceTopPose), source_count = (uint)settings.SourceCount,
                item_count = (uint)settings.ItemCount, stack_pitch = settings.StackPitch, columns = (uint)settings.Columns,
                gap = settings.Gap, row_offset_fraction = settings.RowOffsetFraction,
                placement_corner = CopyPose(settings.PlacementCorner), release_center_x = settings.ReleaseCenterX,
                release_center_y = settings.ReleaseCenterY, release_width = settings.ReleaseWidth,
                release_depth = settings.ReleaseDepth, release_clearance = settings.ReleaseClearance,
                release_yaw_samples = (uint)settings.ReleaseYawSamples, retreat_distance = settings.RetreatDistance,
                settling_delay = settings.SettlingDelay, return_home_each_tile = settings.ReturnHomeEachTile
            };
            reason = null;
            return true;
        }
        catch (ArgumentException exception) { reason = exception.Message; return false; }
    }

    public static bool TryBuildCalibration(FixtureCalibrationData data,
        out SaveFixtureCalibrationRequest request, out string reason)
    {
        request = null;
        try
        {
            Require(data != null && data.BoardTopCorners != null && data.BoardTopCorners.Length == 4,
                "Capture four plank top corners in perimeter order P00, P10, P11, P01.");
            Require(Bounded(data.BoardThickness, 0, 1, false), "Plank thickness must be in (0,1] m.");
            Require(Finite(data.Margin) && data.Margin >= 0, "Plank edge margin must be nonnegative.");
            D3[] p = Array.ConvertAll(data.BoardTopCorners, ReadPoint);
            D3 edgeX = p[1] - p[0], edgeY = p[3] - p[0];
            Require(Bounded(edgeX.Length, 0.001, 10) && Bounded(edgeY.Length, 0.001, 10), "Plank edges must be between 1 mm and 10 m.");
            D3 x = edgeX.Unit, normal = D3.Cross(x, edgeY).Unit;
            Require(Math.Abs(D3.Dot(x, edgeY.Unit)) <= 1e-3 && (p[2] - p[0] - edgeX - edgeY).Length <= 1e-4,
                "Plank corners must form an ordered rectangle (closure within 0.1 mm).");
            Require(normal.Z > 1e-6, "Order plank corners so the top normal points upward.");
            Rotation support = ReadPose(data.SupportCorner);
            D3 corner = ReadPoint(data.SupportCorner.pose.position);
            Require((support.Z - normal).Length <= 1e-6 && Math.Abs(D3.Dot(normal, corner - p[0])) <= 1e-4,
                "The alignment corner must lie on the plank top and share its outward normal.");
            Require(support.X.Z > 1e-6 && support.Y.Z > 1e-6 && support.Z.Z > 1e-6,
                "The plank must slope downhill toward both negative-X and negative-Y alignment stops.");
            for (int index = 0; index < 4; index++)
            {
                D3 edge = p[(index + 1) % 4] - p[index];
                Require(D3.Dot(D3.Cross(normal, edge), corner - p[index]) >= -1e-9,
                    "The alignment corner must be on the plank rectangle.");
            }
            request = new SaveFixtureCalibrationRequest {
                board_top_corners = Array.ConvertAll(p, v => v.Point), board_thickness = data.BoardThickness,
                support_corner = CopyPose(data.SupportCorner), margin = data.Margin
            };
            reason = null;
            return true;
        }
        catch (ArgumentException exception) { reason = exception.Message; return false; }
    }

    /// <summary>Build a right-handed corner frame. Points define direction, not tile dimensions.</summary>
    public static PoseStampedMsg FrameFromPoints(PointMsg corner, PointMsg xPoint, PointMsg yPoint)
    {
        D3 origin = ReadPoint(corner), x = ReadPoint(xPoint) - origin, y = ReadPoint(yPoint) - origin;
        Require(x.Length >= 0.001 && y.Length >= 0.001, "Axis points must be at least 1 mm from the corner.");
        x = x.Unit;
        D3 normal = D3.Cross(x, y);
        Require(normal.Length / y.Length > 1e-3, "Axis points must not be collinear.");
        normal = normal.Unit;
        y = D3.Cross(normal, x);
        return MakePose(origin, Rotation.FromAxes(x, y, normal));
    }

    /// <summary>Explicitly fit an ordered measured plank to one rectangle; retain and display the residual.</summary>
    public static bool TryFitBoard(PointMsg[] measured, float maxResidual, out PointMsg[] fitted,
        out float residual, out string reason)
    {
        fitted = null;
        residual = 0;
        try
        {
            Require(measured != null && measured.Length == 4, "Capture four plank corners before fitting.");
            Require(Bounded(maxResidual, 0, 0.1, false), "The permitted fit residual must be positive and at most 0.1 m.");
            D3[] p = Array.ConvertAll(measured, ReadPoint);
            D3 edgeX = ((p[1]-p[0]) + (p[2]-p[3])) * 0.5;
            D3 edgeY = ((p[3]-p[0]) + (p[2]-p[1])) * 0.5;
            Require(Bounded(edgeX.Length,0.001,10) && Bounded(edgeY.Length,0.001,10), "Plank edges must be between 1 mm and 10 m.");
            Require(Math.Abs(D3.Dot(edgeX.Unit,edgeY.Unit)) < 0.3, "Measured plank edges must describe a near-right angle.");
            D3 normal = D3.Cross(edgeX,edgeY).Unit;
            Require(normal.Z > 1e-6, "Capture the plank corners in perimeter order with its top normal pointing upward.");
            for(int index=0;index<4;index++)
            {
                D3 edge = p[(index+1)%4]-p[index], next=p[(index+2)%4]-p[(index+1)%4];
                Require(edge.Length >= 0.001 && D3.Dot(D3.Cross(edge,next),normal) > 1e-8,
                    "Plank corners must describe a convex ordered perimeter, without repeated or crossed points.");
            }
            // Find the in-plane orientation minimizing squared residuals to the
            // averaged opposite edges, rather than privileging either one.
            D3 u=edgeX.Unit, v=D3.Cross(normal,u);
            double a=edgeX.Length, b=D3.Dot(edgeY,u), c=D3.Dot(edgeY,v);
            double angle=0.5*Math.Atan2(-2*b*c,a*a+c*c-b*b);
            D3 x=u*Math.Cos(angle)+v*Math.Sin(angle);
            D3 y=D3.Cross(normal,x);
            double width=D3.Dot(edgeX,x), depth=D3.Dot(edgeY,y);
            Require(Bounded(width,0.001,10) && Bounded(depth,0.001,10), "The fitted plank dimensions are outside 1 mm–10 m.");
            D3 centre=(p[0]+p[1]+p[2]+p[3])*0.25;
            D3[] rectangle={centre-x*(width/2)-y*(depth/2),centre+x*(width/2)-y*(depth/2),
                centre+x*(width/2)+y*(depth/2),centre-x*(width/2)+y*(depth/2)};
            double maximum=0;
            for(int index=0;index<4;index++)maximum=Math.Max(maximum,(rectangle[index]-p[index]).Length);
            residual=(float)maximum;
            Require(maximum <= maxResidual, "Plank fit exceeds the permitted residual; recapture the corners.");
            fitted=Array.ConvertAll(rectangle,point=>point.Point);
            reason=null;
            return true;
        }
        catch(ArgumentException exception){reason=exception.Message;return false;}
    }

    /// <summary>Project measured stop axes onto the fitted plank, retaining exactly its outward normal.</summary>
    public static PoseStampedMsg ProjectFrameToBoard(FixtureCalibrationData data, PointMsg origin,
        PointMsg xPoint, PointMsg yPoint)
    {
        Require(data?.BoardTopCorners != null && data.BoardTopCorners.Length == 4, "Fit the plank rectangle before defining its alignment axes.");
        D3[] p=Array.ConvertAll(data.BoardTopCorners,ReadPoint);
        D3 boardX=p[1]-p[0], boardY=p[3]-p[0];
        Require(boardX.Length>=0.001 && boardY.Length>=0.001 &&
            Math.Abs(D3.Dot(boardX.Unit,boardY.Unit))<=1e-3 && (p[2]-p[0]-boardX-boardY).Length<=1e-4,
            "Fit the plank rectangle before defining its alignment axes.");
        D3 normal=D3.Cross(boardX,boardY).Unit;
        Require(normal.Z>1e-6,"The fitted plank top normal must point upward.");
        D3 raw=ReadPoint(origin), x=ReadPoint(xPoint)-raw, y=ReadPoint(yPoint)-raw;
        D3 corner=raw-normal*D3.Dot(raw-p[0],normal);
        x=x-normal*D3.Dot(x,normal); y=y-normal*D3.Dot(y,normal);
        Require(x.Length>=0.001 && y.Length>=0.001,"Alignment axis points must be distinct on the plank plane.");
        x=x.Unit;y=y.Unit;
        Require(Math.Abs(D3.Dot(x,y))<0.15,"Alignment +X and +Y must follow perpendicular stop arms.");
        Require(D3.Dot(D3.Cross(x,y),normal)>0,"Alignment +X and +Y must run into the receiving area with the plank normal pointing out.");
        return MakePose(corner,Rotation.FromAxes(x,D3.Cross(normal,x),normal));
    }

    /// <summary>The horizontal stack frame keeps the measured yaw in its collision footprint.</summary>
    public static PoseStampedMsg SourceTopFrame(PointMsg position, double yawRadians)
    {
        Require(Finite(yawRadians), "Stack yaw must be finite.");
        return MakePose(ReadPoint(position), new QuaternionMsg(0, 0, Math.Sin(yawRadians / 2), Math.Cos(yawRadians / 2)));
    }

    public static PoseStampedMsg SourceObjectPose(FixtureTilingSettings settings, int index)
    {
        Require(index >= 0 && index < settings.SourceCount, "Source index is outside the stack.");
        return OffsetPose(settings.SourceTopPose, 0, 0,
            -settings.ItemThickness / 2.0 - index * (settings.StackPitch == 0 ? settings.ItemThickness : settings.StackPitch));
    }

    public static PoseStampedMsg PlacementObjectPose(FixtureTilingSettings settings, int index)
    {
        Require(settings.Columns > 0 && index >= 0 && index < settings.ItemCount, "Destination index is outside the pattern.");
        int row = index / settings.Columns, column = index % settings.Columns;
        double pitch = (double)settings.ItemSize + settings.Gap;
        return OffsetPose(settings.PlacementCorner,
            settings.ItemSize / 2.0 + column * pitch + (row % 2) * settings.RowOffsetFraction * pitch,
            settings.ItemSize / 2.0 + row * pitch, -settings.ItemThickness / 2.0);
    }

    public static PointMsg PointInFrame(PoseStampedMsg frame, double x, double y, double z)
    {
        Rotation rotation = ReadPose(frame);
        return (ReadPoint(frame.pose.position) + rotation.X * x + rotation.Y * y + rotation.Z * z).Point;
    }

    public static PoseStampedMsg OffsetPose(PoseStampedMsg frame, double x, double y, double z)
    {
        PointMsg point = PointInFrame(frame, x, y, z);
        return MakePose(ReadPoint(point), ReadPose(frame).Quaternion);
    }

    public static PoseStampedMsg CopyPose(PoseStampedMsg frame) => MakePose(ReadPoint(frame?.pose?.position), ReadPose(frame).Quaternion);

    public static bool ValidToken(string token)
    {
        if (string.IsNullOrEmpty(token) || token.Length > 128) return false;
        foreach (char c in token)
            if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == ':')) return false;
        return true;
    }

    private static PoseStampedMsg MakePose(D3 position, QuaternionMsg rotation) =>
        new PoseStampedMsg(new HeaderMsg { frame_id = FrameId }, new PoseMsg(position.Point, rotation));
    private static D3 ReadPoint(PointMsg point)
    {
        Require(point != null && Finite(point.x) && Finite(point.y) && Finite(point.z), "Capture finite measured points first.");
        return new D3(point.x, point.y, point.z);
    }
    private static Rotation ReadPose(PoseStampedMsg frame)
    {
        Require(frame?.header != null && frame.pose != null && frame.header.frame_id == FrameId &&
            frame.header.stamp != null && frame.header.stamp.sec == 0 && frame.header.stamp.nanosec == 0,
            "Capture static object frames in ur_base_link.");
        ReadPoint(frame.pose.position);
        QuaternionMsg q = frame.pose.orientation;
        Require(q != null && Finite(q.x) && Finite(q.y) && Finite(q.z) && Finite(q.w), "Object frame rotations must be finite.");
        double length = Math.Sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        Require(Math.Abs(length - 1) <= 1e-6, "Object frame rotations must be unit quaternions.");
        return new Rotation(new QuaternionMsg(q.x / length, q.y / length, q.z / length, q.w / length));
    }
    private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    private static bool Bounded(double value, double minimum, double maximum, bool inclusiveMinimum = true) =>
        Finite(value) && (inclusiveMinimum ? value >= minimum : value > minimum) && value <= maximum;
    private static void Require(bool condition, string reason) { if (!condition) throw new ArgumentException(reason); }

    private readonly struct D3
    {
        public readonly double X, Y, Z;
        public D3(double x, double y, double z) { X = x; Y = y; Z = z; }
        public double Length => Math.Sqrt(Dot(this, this));
        public D3 Unit => this * (1.0 / Length);
        public PointMsg Point => new PointMsg(X, Y, Z);
        public static D3 operator +(D3 a, D3 b) => new D3(a.X+b.X, a.Y+b.Y, a.Z+b.Z);
        public static D3 operator -(D3 a, D3 b) => new D3(a.X-b.X, a.Y-b.Y, a.Z-b.Z);
        public static D3 operator *(D3 a, double scale) => new D3(a.X*scale, a.Y*scale, a.Z*scale);
        public static double Dot(D3 a, D3 b) => a.X*b.X + a.Y*b.Y + a.Z*b.Z;
        public static D3 Cross(D3 a, D3 b) => new D3(a.Y*b.Z-a.Z*b.Y, a.Z*b.X-a.X*b.Z, a.X*b.Y-a.Y*b.X);
    }

    private readonly struct Rotation
    {
        public readonly QuaternionMsg Quaternion;
        public readonly D3 X, Y, Z;
        public Rotation(QuaternionMsg q)
        {
            Quaternion = q;
            X = new D3(1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y+q.z*q.w), 2*(q.x*q.z-q.y*q.w));
            Y = new D3(2*(q.x*q.y-q.z*q.w), 1-2*(q.x*q.x+q.z*q.z), 2*(q.y*q.z+q.x*q.w));
            Z = new D3(2*(q.x*q.z+q.y*q.w), 2*(q.y*q.z-q.x*q.w), 1-2*(q.x*q.x+q.y*q.y));
        }
        public static QuaternionMsg FromAxes(D3 x, D3 y, D3 z)
        {
            double trace = x.X + y.Y + z.Z;
            double qx, qy, qz, qw, scale;
            if (trace > 0) { scale = Math.Sqrt(trace+1)*2; qw=scale/4; qx=(y.Z-z.Y)/scale; qy=(z.X-x.Z)/scale; qz=(x.Y-y.X)/scale; }
            else if (x.X > y.Y && x.X > z.Z) { scale=Math.Sqrt(1+x.X-y.Y-z.Z)*2; qw=(y.Z-z.Y)/scale; qx=scale/4; qy=(y.X+x.Y)/scale; qz=(z.X+x.Z)/scale; }
            else if (y.Y > z.Z) { scale=Math.Sqrt(1+y.Y-x.X-z.Z)*2; qw=(z.X-x.Z)/scale; qx=(y.X+x.Y)/scale; qy=scale/4; qz=(z.Y+y.Z)/scale; }
            else { scale=Math.Sqrt(1+z.Z-x.X-y.Y)*2; qw=(x.Y-y.X)/scale; qx=(z.X+x.Z)/scale; qy=(z.Y+y.Z)/scale; qz=scale/4; }
            double norm=Math.Sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
            return new QuaternionMsg(qx/norm,qy/norm,qz/norm,qw/norm);
        }
    }
}
