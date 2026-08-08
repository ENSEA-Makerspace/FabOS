<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Contract step of the polymorphic-Reservation refactor: drops machineId and
 * placeId now that nothing reads or writes them, and makes the polymorphic pair
 * mandatory.
 *
 * ⚠️ Deploy order is the REVERSE of the expand migration (Version20260724100000).
 * The application code that stopped mapping machine/place must already be live
 * when this runs — an entity narrower than its table is always safe, an entity
 * wider than its table 500s every page that hydrates it. Do not run this against
 * a checkout still carrying those associations.
 *
 * Dropping the FKs also drops their ON DELETE CASCADE, which is what used to
 * clean up bookings of a deleted resource. AdminController::deletePlace now
 * calls ReservationRepository::cancelUpcomingForReservable() to replace it;
 * anything that gains a delete path later must do the same.
 */
final class Version20260724160000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Drop RESERVATION.machineId/placeId and make reservableType/reservableId mandatory.';
    }

    public function up(Schema $schema): void
    {
        // Idempotent re-backfill: covers any row written between the expand
        // migration and the code deploy. Must happen while the columns exist.
        $this->addSql("UPDATE RESERVATION r JOIN MACHINE m ON m.id = r.machineId SET r.reservableType = 'machine', r.reservableId = m.id, r.reservableLabel = COALESCE(r.reservableLabel, m.nom) WHERE r.reservableType IS NULL AND r.machineId IS NOT NULL");
        $this->addSql("UPDATE RESERVATION r JOIN PLACE p ON p.id = r.placeId SET r.reservableType = 'place', r.reservableId = p.id, r.reservableLabel = COALESCE(r.reservableLabel, p.nom) WHERE r.reservableType IS NULL AND r.placeId IS NOT NULL");

        $this->addSql('ALTER TABLE RESERVATION DROP FOREIGN KEY fk_reservation_machine');
        $this->addSql('ALTER TABLE RESERVATION DROP FOREIGN KEY FK_RESERVATION_PLACE');
        $this->addSql('ALTER TABLE RESERVATION DROP machineId, DROP placeId');

        $this->addSql('ALTER TABLE RESERVATION MODIFY reservableType VARCHAR(20) NOT NULL, MODIFY reservableId INT NOT NULL');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('ALTER TABLE RESERVATION MODIFY reservableType VARCHAR(20) DEFAULT NULL, MODIFY reservableId INT DEFAULT NULL');
        $this->addSql('ALTER TABLE RESERVATION ADD machineId INT DEFAULT NULL, ADD placeId INT DEFAULT NULL');

        // Re-derive the FK columns from the polymorphic pair before re-adding the
        // constraints, so the rollback lands on valid data.
        $this->addSql("UPDATE RESERVATION SET machineId = reservableId WHERE reservableType = 'machine'");
        $this->addSql("UPDATE RESERVATION SET placeId = reservableId WHERE reservableType = 'place'");

        $this->addSql('ALTER TABLE RESERVATION ADD CONSTRAINT fk_reservation_machine FOREIGN KEY (machineId) REFERENCES MACHINE (id) ON DELETE CASCADE');
        $this->addSql('ALTER TABLE RESERVATION ADD CONSTRAINT FK_RESERVATION_PLACE FOREIGN KEY (placeId) REFERENCES PLACE (id) ON DELETE CASCADE');
    }
}
