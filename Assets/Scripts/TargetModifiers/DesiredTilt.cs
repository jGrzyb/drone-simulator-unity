public struct DesiredTilt
{
    public float desiredRoll;
    public float desiredPitch;

    public DesiredTilt(float roll, float pitch)
    {
        desiredRoll = roll;
        desiredPitch = pitch;
    }

    public static DesiredTilt operator *(DesiredTilt tilt, float multiplier)
    {
        return new DesiredTilt(tilt.desiredRoll * multiplier, tilt.desiredPitch * multiplier);
    }
}