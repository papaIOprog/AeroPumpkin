use pumpkin_nbt::compound::NbtCompound;
use pumpkin_util::math::position::BlockPos;

use super::BlockEntity;

/// A block entity of a mod's: Pumpkin runs none of its logic, so its NBT
/// is kept as it came and written back as it was.
pub struct ForeignBlockEntity {
    /// The type, as the mod names it: `namespace:path`.
    kind: &'static str,
    /// The type's id in the clients' registry; 0 where none knows it.
    type_id: u32,
    position: BlockPos,
    nbt: NbtCompound,
}

impl ForeignBlockEntity {
    #[must_use]
    pub fn new(kind: &'static str, type_id: u16, nbt: NbtCompound, position: BlockPos) -> Self {
        Self {
            kind,
            type_id: u32::from(type_id),
            position,
            nbt,
        }
    }
}

impl BlockEntity for ForeignBlockEntity {
    fn write_nbt(&self, nbt: &mut NbtCompound) {
        // The id and the position are written around this.
        for (tag, value) in &self.nbt.child_tags {
            if !matches!(tag.as_ref(), "id" | "x" | "y" | "z") {
                nbt.put(tag, value.clone());
            }
        }
    }

    fn from_nbt(nbt: &NbtCompound, position: BlockPos) -> Self
    where
        Self: Sized,
    {
        let id = nbt.get_string("id").unwrap_or("");
        Self::new(
            Box::leak(id.to_owned().into_boxed_str()),
            super::block_entity_type_id(id).unwrap_or(0),
            nbt.clone(),
            position,
        )
    }

    fn resource_location(&self) -> &'static str {
        self.kind
    }

    fn get_position(&self) -> BlockPos {
        self.position
    }

    fn get_id(&self) -> u32 {
        self.type_id
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_mods_block_entity_keeps_its_nbt() {
        let position = BlockPos::new(1, 2, 3);
        let mut nbt = NbtCompound::new();
        nbt.put_string("id", "test:holder".to_owned());
        nbt.put_int("x", 1);
        nbt.put_int("y", 2);
        nbt.put_int("z", 3);
        nbt.put_string("Owner", "someone".to_owned());
        nbt.put_int("Count", 7);
        let entity = ForeignBlockEntity::from_nbt(&nbt, position);

        let mut written = NbtCompound::new();
        entity.write_internal(&mut written);
        assert_eq!(written.get_string("id"), Some("test:holder"));
        assert_eq!(
            (
                written.get_int("x"),
                written.get_int("y"),
                written.get_int("z")
            ),
            (Some(1), Some(2), Some(3))
        );
        assert_eq!(written.get_string("Owner"), Some("someone"));
        assert_eq!(written.get_int("Count"), Some(7));
        assert_eq!(entity.resource_location(), "test:holder");
        // Without a profile the type is in no registry the server knows.
        assert_eq!(entity.get_id(), 0);
    }
}
