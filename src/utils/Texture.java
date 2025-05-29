package utils;

import static org.lwjgl.opengl.GL46C.*;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;

import javax.imageio.ImageIO;

import org.lwjgl.stb.STBImage;

public class Texture {
	public int id;
	public int width;
	public int height;

	public Texture(int width, int height) {
		this.width = width;
		this.height = height;
		id = glCreateTextures(GL_TEXTURE_2D);
		bind();
		glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, this.width, this.height);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		unbind();

	}

	public Texture(String path) throws IOException {
		System.out.println("Loading texture: " + path);

		int width[] = new int[1];
		int height[] = new int[1];
		int channels[] = new int[1];
		ByteBuffer buff = STBImage.stbi_load(path, width, height, channels, 4);

		this.width = width[0];
		this.height = height[0];
		
		int levels = 1;
		while ((this.width|this.height) >> levels != 0) levels += 1;

		id = glCreateTextures(GL_TEXTURE_2D);
		bind();
		glTexStorage2D(GL_TEXTURE_2D, levels, GL_RGBA8, this.width, this.height);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, this.width, this.height, GL_RGBA, GL_UNSIGNED_BYTE, (ByteBuffer) buff);
		glGenerateTextureMipmap(id);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY, 8f);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		unbind();

		STBImage.stbi_image_free(buff);
	}

	public Texture(BufferedImage image) throws IOException {
		ByteArrayOutputStream baos = new ByteArrayOutputStream();
		ImageIO.write(image, "png", baos);
		ByteBuffer buff = ByteBuffer.wrap(baos.toByteArray());
		this.width = image.getWidth();
		this.height = image.getHeight();

		id = glCreateTextures(GL_TEXTURE_2D);
		bind();
		glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, this.width, this.height);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, this.width, this.height, GL_RGBA, GL_UNSIGNED_BYTE, (ByteBuffer) buff);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		unbind();
	}

	public void bind() {
		glBindTexture(GL_TEXTURE_2D, id);
	}

	public void unbind() {
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	public void bindAsTexture(int unit) {
		glActiveTexture(GL_TEXTURE0 + unit);
		glBindTexture(GL_TEXTURE_2D, id);
	}

	public void unbindAsTexture(int unit) {
		glActiveTexture(GL_TEXTURE0 + unit);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	public void delete() {
		glDeleteTextures(id);
	}
}