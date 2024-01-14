package org.frcteam3539.Byte_Swerve_Lib.io.json;

import com.google.gson.*;

import java.lang.reflect.Type;

import org.frcteam3539.Byte_Swerve_Lib.util.InterpolatingDouble;

public final class InterpolatingDoubleJsonHandler implements JsonDeserializer<InterpolatingDouble>, JsonSerializer<InterpolatingDouble> {
    @Override
    public JsonElement serialize(InterpolatingDouble src, Type typeOfSrc, JsonSerializationContext context) {
        return new JsonPrimitive(src.value);
    }

    @Override
    public InterpolatingDouble deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context) throws JsonParseException {
        return new InterpolatingDouble(json.getAsDouble());
    }
}
