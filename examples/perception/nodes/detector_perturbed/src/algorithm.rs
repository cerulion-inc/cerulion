use cerulion_core::prelude::NodeError;

// Fixed-size scalar state; image bytes remain borrowed from shared memory.
#[derive(Clone, Copy)]
pub struct Region {
    pub left: usize,
    pub top: usize,
    pub right: usize,
    pub bottom: usize,
}

// Three intensity bands label the generated rectangles. This is an image
// operation for a regression tutorial, not an ML model or semantic classifier.
pub fn detect(
    data: &[u8],
    width: u32,
    height: u32,
    step: u32,
    encoding: &str,
) -> Result<[Option<Region>; 3], NodeError> {
    let (width, height, step) = (width as usize, height as usize, step as usize);
    if encoding != "mono8"
        || width == 0
        || height == 0
        || step < width
        || height.checked_mul(step) != Some(data.len())
    {
        return Err(NodeError::Logic(
            "expected a nonempty mono8 image with consistent stride and data length".into(),
        ));
    }
    let mut regions: [Option<Region>; 3] = [None; 3];
    for (y, row) in data.chunks_exact(step).enumerate() {
        for (x, &value) in row[..width].iter().enumerate() {
            let class = match value {
                224..=255 => 0,
                192..=223 => 1,
                160..=191 => 2,
                _ => continue,
            };
            let bounds = regions[class].get_or_insert(Region {
                left: x,
                top: y,
                right: x,
                bottom: y,
            });
            bounds.left = bounds.left.min(x);
            bounds.top = bounds.top.min(y);
            bounds.right = bounds.right.max(x);
            bounds.bottom = bounds.bottom.max(y);
        }
    }
    Ok(regions)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bands_produce_handwritten_bounds_and_ignore_row_padding() {
        let data = [230, 230, 0, 255, 0, 204, 179, 255];
        let result = detect(&data, 3, 2, 4, "mono8").unwrap();
        let bounds = result.map(|r| r.map(|r| (r.left, r.top, r.right, r.bottom)));
        assert_eq!(
            bounds,
            [Some((0, 0, 1, 0)), Some((1, 1, 1, 1)), Some((2, 1, 2, 1))]
        );
        assert!(detect(&[159, 0], 2, 1, 2, "mono8")
            .unwrap()
            .iter()
            .all(Option::is_none));
    }

    #[test]
    fn invalid_images_are_rejected_before_scanning() {
        for (data, width, height, step, encoding) in [
            (&[][..], 0, 1, 1, "mono8"),
            (&[][..], 1, 0, 1, "mono8"),
            (&[0][..], 2, 1, 1, "mono8"),
            (&[0][..], 1, 2, 1, "mono8"),
            (&[0][..], 1, 1, 1, "rgb8"),
        ] {
            assert!(detect(data, width, height, step, encoding).is_err());
        }
    }
}
