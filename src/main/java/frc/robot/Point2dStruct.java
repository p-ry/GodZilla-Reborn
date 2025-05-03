package frc.robot;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.awt.geom.Point2D;

public class Point2dStruct implements Struct<Point2D.Double> {

    @Override
    public String getTypeName() {
        return "Point2D.Double";
    }
    public static final Point2dStruct instance = new Point2dStruct();

    @Override
    public Class<Point2D.Double> getTypeClass() {
        return Point2D.Double.class;
    }

    @Override
    public String getTypeString() {
        return "point2d";
    }

    @Override
    public int getSize() {
        return Double.BYTES * 2;
    }

    @Override
    public String getSchema() {
        return "double x;double y";
    }

    @Override
    public Point2D.Double unpack(ByteBuffer bb) {
        bb.order(ByteOrder.LITTLE_ENDIAN);
        return new Point2D.Double(bb.getDouble(), bb.getDouble());
    }

    @Override
    public void pack(ByteBuffer bb, Point2D.Double value) {
        bb.order(ByteOrder.LITTLE_ENDIAN);
        bb.putDouble(value.getX());
        bb.putDouble(value.getY());
    }
}

