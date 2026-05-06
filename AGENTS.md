# Agent Notes

## README Media Gallery Formatting

When editing the README media gallery, keep the current format:

- Use a simple centered thumbnail array in Markdown/HTML.
- Do not use carousel behavior, JavaScript, previous/next arrows, or custom anchor navigation.
- Display gallery thumbnails as two rows when there are seven images: four thumbnails on the first row and three thumbnails below.
- Each thumbnail should be clickable and link to the original full-size image in `media/photos/`.
- Visible gallery thumbnails should use generated PNG files from `media/photos/thumbs/`, not the original full-size photos directly.
- Gallery thumbnails should be `width="180"` in the README.
- Gallery thumbnail PNGs should be generated at 2x display resolution, currently `360x250`, so previews stay sharp when downscaled.
- Thumbnail images must not be cropped or zoomed. Generate them with the full source image contained inside a fixed thumbnail canvas, with subtle translucent padding/background.
- Thumbnail corners should be lightly rounded, not heavily rounded.
- The main top hero photo should also use a generated rounded thumbnail from `media/photos/thumbs/hero-robot.png`.
- The hero photo should have rounded corners only, with no padded translucent background.
- The hero photo should be displayed at `width="520"` and link to the original full-size image.
- The hero PNG should be generated at 2x display width, currently `1040px` wide, for a sharper preview.

Ignore temporary files such as `*.TMP` when updating the gallery.
