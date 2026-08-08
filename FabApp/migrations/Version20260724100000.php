<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/**
 * Expand step of the polymorphic-Reservation refactor: a reservation stops
 * pointing at "a machine or a place" through two nullable FKs and starts
 * pointing at any reservable through a (reservableType, reservableId) pair,
 * so people — and later any other resource — become bookable without a third
 * nullable column.
 *
 * reservableLabel is a snapshot of the resource name taken at booking time. It
 * keeps admin search a plain LIKE (no join to resolve a name) and keeps history
 * readable after a machine or place is deleted — with no FK, nothing cascades
 * those rows away any more.
 *
 * Additive only: machineId/placeId stay in place and keep being written, so the
 * currently deployed code runs unchanged against this schema. The contract step
 * (NOT NULL + dropping both FK columns) is a separate migration, deployed only
 * after the code that reads them is gone.
 */
final class Version20260724100000 extends AbstractMigration
{
    public function getDescription(): string
    {
        return 'Add polymorphic reservableType/reservableId/reservableLabel to RESERVATION and backfill from machineId/placeId.';
    }

    public function up(Schema $schema): void
    {
        $this->addSql('ALTER TABLE RESERVATION ADD reservableType VARCHAR(20) DEFAULT NULL, ADD reservableId INT DEFAULT NULL, ADD reservableLabel VARCHAR(190) DEFAULT NULL');

        $this->addSql("UPDATE RESERVATION r JOIN MACHINE m ON m.id = r.machineId SET r.reservableType = 'machine', r.reservableId = m.id, r.reservableLabel = m.nom WHERE r.machineId IS NOT NULL");
        $this->addSql("UPDATE RESERVATION r JOIN PLACE p ON p.id = r.placeId SET r.reservableType = 'place', r.reservableId = p.id, r.reservableLabel = p.nom WHERE r.placeId IS NOT NULL");

        $this->addSql('CREATE INDEX IDX_RESERVATION_RESERVABLE ON RESERVATION (reservableType, reservableId, dateDebut)');
    }

    public function down(Schema $schema): void
    {
        $this->addSql('DROP INDEX IDX_RESERVATION_RESERVABLE ON RESERVATION');
        $this->addSql('ALTER TABLE RESERVATION DROP reservableType, DROP reservableId, DROP reservableLabel');
    }
}
