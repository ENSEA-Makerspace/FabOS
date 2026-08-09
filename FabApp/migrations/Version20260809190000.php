<?php

declare(strict_types=1);

namespace DoctrineMigrations;

use Doctrine\DBAL\Schema\Schema;
use Doctrine\Migrations\AbstractMigration;

/** S118: the final booking-policy field, enforced by BookingVerbService. */
final class Version20260809190000 extends AbstractMigration
{
    public function isTransactional(): bool { return false; }
    public function getDescription(): string { return 'Add the cancellation/reschedule notice to booking policies.'; }
    public function up(Schema $schema): void { $this->addSql('ALTER TABLE BOOKING_POLICY ADD cancellationNoticeMinutes INT DEFAULT NULL'); }
    public function down(Schema $schema): void { $this->addSql('ALTER TABLE BOOKING_POLICY DROP cancellationNoticeMinutes'); }
}
