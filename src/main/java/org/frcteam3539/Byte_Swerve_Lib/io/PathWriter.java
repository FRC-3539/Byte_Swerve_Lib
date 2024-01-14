package org.frcteam3539.Byte_Swerve_Lib.io;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonIOException;
import com.google.gson.JsonObject;
import edu.wpi.first.math.geometry.Rotation2d;
import org.ejml.simple.SimpleMatrix;
import org.frcteam3539.Byte_Swerve_Lib.control.Path;
import org.frcteam3539.Byte_Swerve_Lib.control.PathSegment;
import org.frcteam3539.Byte_Swerve_Lib.io.json.InterpolatingDoubleJsonHandler;
import org.frcteam3539.Byte_Swerve_Lib.io.json.PathSegmentJsonHandler;
import org.frcteam3539.Byte_Swerve_Lib.io.json.Rotation2JsonHandler;
import org.frcteam3539.Byte_Swerve_Lib.io.json.SimpleMatrixJsonHandler;
import org.frcteam3539.Byte_Swerve_Lib.util.InterpolatingDouble;

import java.io.Flushable;
import java.io.IOException;
import java.io.Writer;

public final class PathWriter implements AutoCloseable, Flushable {
    private final Gson gson;
    private final Writer out;

    public PathWriter(Writer out) {
        this.gson = new GsonBuilder()
                .registerTypeAdapter(InterpolatingDouble.class, new InterpolatingDoubleJsonHandler())
                .registerTypeHierarchyAdapter(PathSegment.class, new PathSegmentJsonHandler())
                .registerTypeAdapter(Rotation2d.class, new Rotation2JsonHandler())
                .registerTypeAdapter(SimpleMatrix.class, new SimpleMatrixJsonHandler())
                .enableComplexMapKeySerialization()
                .create();
        this.out = out;
    }

    public void write(Path path) throws IOException {
        try {
            JsonObject root = new JsonObject();
            root.add("segments", gson.toJsonTree(path.getSegments()));
            root.add("rotations", gson.toJsonTree(path.getRotationMap()));
            gson.toJson(root, out);
        } catch (JsonIOException e) {
            throw new IOException(e);
        }
    }

    @Override
    public void flush() throws IOException {
        out.flush();
    }

    @Override
    public void close() throws IOException {
        out.close();
    }
}
