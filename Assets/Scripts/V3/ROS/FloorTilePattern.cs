using System;
using UnityEngine;

/// <summary>A row-major tiling in metres, with its origin at the first tile's floor corner.</summary>
public readonly struct FloorTilePattern
{
    public const int MinimumTileCount = 1;
    public const int MaximumTileCount = 64;
    public const int MinimumMortarMillimeters = 5;
    public const int MaximumMortarMillimeters = 15;

    public int TileCount { get; }
    public int Rows { get; }
    public int Columns { get; }
    public float TileSide { get; }
    public float TileThickness { get; }
    public int OffsetIndex { get; }
    public int MortarMillimeters { get; }
    /// <summary>Center of the occupied footprint, in local X/Z.</summary>
    public Vector2 BoundsCenter { get; }

    private readonly float _pitch;

    public FloorTilePattern(int tileCount, float tileSide, float tileThickness, int offsetIndex, int mortarMillimeters)
    {
        if (tileCount < MinimumTileCount || tileCount > MaximumTileCount)
            throw new ArgumentOutOfRangeException(nameof(tileCount));
        if (tileSide <= 0f || float.IsNaN(tileSide) || float.IsInfinity(tileSide))
            throw new ArgumentOutOfRangeException(nameof(tileSide));
        if (tileThickness <= 0f || float.IsNaN(tileThickness) || float.IsInfinity(tileThickness))
            throw new ArgumentOutOfRangeException(nameof(tileThickness));
        if (offsetIndex < 0 || offsetIndex > 2)
            throw new ArgumentOutOfRangeException(nameof(offsetIndex));
        if (mortarMillimeters < MinimumMortarMillimeters || mortarMillimeters > MaximumMortarMillimeters)
            throw new ArgumentOutOfRangeException(nameof(mortarMillimeters));

        TileCount = tileCount;
        Columns = (int)Math.Ceiling(Math.Sqrt(tileCount));
        Rows = (tileCount + Columns - 1) / Columns;
        TileSide = tileSide;
        TileThickness = tileThickness;
        OffsetIndex = offsetIndex;
        MortarMillimeters = mortarMillimeters;
        _pitch = tileSide + mortarMillimeters * .001f;

        float width = 0f;
        for (int row = 0; row < Rows; row++)
        {
            int tilesInRow = Math.Min(Columns, tileCount - row * Columns);
            float edge = RowOffset(row, offsetIndex, tileSide) + (tilesInRow - 1) * _pitch + tileSide;
            width = Math.Max(width, edge);
        }
        float length = (Rows - 1) * _pitch + tileSide;
        if (float.IsInfinity(width) || float.IsInfinity(length))
            throw new ArgumentOutOfRangeException(nameof(tileSide));
        BoundsCenter = new Vector2(width * .5f, length * .5f);
    }

    public Vector3 GetTileCenter(int index)
    {
        if (index < 0 || index >= TileCount) throw new ArgumentOutOfRangeException(nameof(index));
        int row = index / Columns;
        int column = index % Columns;
        float halfSide = TileSide * .5f;
        return new Vector3(
            halfSide + column * _pitch + RowOffset(row, OffsetIndex, TileSide),
            TileThickness * .5f,
            halfSide + row * _pitch);
    }

    private static float RowOffset(int row, int offsetIndex, float tileSide)
    {
        switch (offsetIndex)
        {
            case 1: return tileSide * ((row % 3) / 3f);
            case 2: return tileSide * ((row % 2) / 2f);
            default: return 0f;
        }
    }
}
