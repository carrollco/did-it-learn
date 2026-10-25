"""Rasterize our locally generated SVG using installed system libraries, no packages or network."""
import ctypes as C
import ctypes.util
import sys


def raster(svg_path, png_path, width, height, jpeg_path=None):
    cairo = C.CDLL(ctypes.util.find_library("cairo"))
    rsvg = C.CDLL(ctypes.util.find_library("rsvg-2"))
    gobject = C.CDLL(ctypes.util.find_library("gobject-2.0"))
    cairo.cairo_image_surface_create.argtypes = [C.c_int, C.c_int, C.c_int]
    cairo.cairo_image_surface_create.restype = C.c_void_p
    cairo.cairo_create.argtypes = [C.c_void_p]
    cairo.cairo_create.restype = C.c_void_p
    cairo.cairo_surface_write_to_png.argtypes = [C.c_void_p, C.c_char_p]
    cairo.cairo_surface_write_to_png.restype = C.c_int
    cairo.cairo_destroy.argtypes = [C.c_void_p]
    cairo.cairo_surface_destroy.argtypes = [C.c_void_p]
    rsvg.rsvg_handle_new_from_data.argtypes = [C.c_char_p, C.c_size_t, C.c_void_p]
    rsvg.rsvg_handle_new_from_data.restype = C.c_void_p
    rsvg.rsvg_handle_render_cairo.argtypes = [C.c_void_p, C.c_void_p]
    rsvg.rsvg_handle_render_cairo.restype = C.c_int
    gobject.g_object_unref.argtypes = [C.c_void_p]
    data = open(svg_path, "rb").read()
    handle = rsvg.rsvg_handle_new_from_data(data, len(data), None)
    if not handle:
        raise RuntimeError("system SVG reader rejected the image")
    surface = cairo.cairo_image_surface_create(0, int(width), int(height))
    context = cairo.cairo_create(surface)
    try:
        if not rsvg.rsvg_handle_render_cairo(handle, context):
            raise RuntimeError("system SVG renderer failed")
        status = cairo.cairo_surface_write_to_png(surface, png_path.encode())
        if status:
            raise RuntimeError(f"PNG write failed, Cairo status {status}")
        if jpeg_path:
            pixbuf = C.CDLL(ctypes.util.find_library("gdk_pixbuf-2.0"))
            pixbuf.gdk_pixbuf_new_from_file.argtypes = [C.c_char_p, C.c_void_p]
            pixbuf.gdk_pixbuf_new_from_file.restype = C.c_void_p
            pixbuf.gdk_pixbuf_savev.argtypes = [C.c_void_p, C.c_char_p, C.c_char_p, C.POINTER(C.c_char_p), C.POINTER(C.c_char_p), C.c_void_p]
            pixbuf.gdk_pixbuf_savev.restype = C.c_int
            image = pixbuf.gdk_pixbuf_new_from_file(png_path.encode(), None)
            if not image:
                raise RuntimeError("PNG to JPEG conversion failed")
            try:
                keys = (C.c_char_p * 2)(b"quality", None)
                values = (C.c_char_p * 2)(b"85", None)
                if not pixbuf.gdk_pixbuf_savev(image, jpeg_path.encode(), b"jpeg", keys, values, None):
                    raise RuntimeError("JPEG write failed")
            finally:
                gobject.g_object_unref(image)
    finally:
        cairo.cairo_destroy(context)
        cairo.cairo_surface_destroy(surface)
        gobject.g_object_unref(handle)


if __name__ == "__main__":
    raster(*sys.argv[1:])
